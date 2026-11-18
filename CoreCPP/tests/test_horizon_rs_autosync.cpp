#include "FrameWarpCalculator.hpp"
#include "OpticalSync.hpp"
#include "TrajectoryEngine.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

void testHorizonLock360() {
    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = 1920;
    hdr.height = 1080;
    hdr.nominal_fps = 30.0;

    // Camera rolls 180 degrees around +Z_cam over 3 seconds at 90Hz
    const int nImu = 270;
    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    const double rollRate = kPi / 3.0; // rad/s

    for (int i = 0; i < nImu; ++i) {
        const double t = 10.0 + i / 90.0;
        const double theta = rollRate * (t - 10.0);
        // iOS LandscapeLeft: (-y, -x, -z). To get +rollRate around Z_cam, gz_ios = -rollRate
        gyro.push_back({t, 0.0, 0.0, -rollRate});
        // World gravity is +Y_cam = (0, 1, 0) in cam at t=0.
        // In camera frame rotated by +theta around Z_cam, gravity = (-sin(theta), cos(theta), 0)
        // Inverse of (-y, -x, -z) -> a_ios = (-ay_cam, -ax_cam, -az_cam)
        const double gxCam = std::sin(theta);
        const double gyCam = std::cos(theta);
        accel.push_back({t, static_cast<float>(-gyCam), static_cast<float>(-gxCam), 0.0f});
    }

    std::vector<gf::GFFrameRecord> frames;
    for (int f = 0; f < 90; ++f) {
        frames.push_back({static_cast<uint32_t>(f), 10.0 + f / 30.0, 0.008f, 100.0f, 0.7f, 0.0152f});
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel);

    gf::StabilizerConfig cfg;
    cfg.pitchSmoothness = 0.2;
    cfg.yawSmoothness = 0.2;
    cfg.rollSmoothness = 0.2;
    cfg.horizonLockEnabled = true;
    cfg.horizonLockAmount = 1.0;

    const auto traj = engine.solve(frames, cfg);
    assert(traj.valid);

    // At 45 frames (90 deg roll) and 89 frames (180 deg roll), camera's +Y_cam (down) must stay aligned with world +Y
    for (size_t idx : {size_t(0), size_t(45), size_t(89)}) {
        const gf::Vec3 downW = traj.smoothFrameQuats[idx].rotateVector({0.0, 1.0, 0.0}).normalized();
        assert(downW.y > 0.98);
    }
}

void testRollingShutterSlices() {
    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = 1920;
    hdr.height = 1080;
    hdr.nominal_fps = 30.0;

    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    for (int i = 0; i < 90; ++i) {
        const double t = 1.0 + i / 90.0;
        // Fast yaw 2.0 rad/s -> gx_ios = -2.0
        gyro.push_back({t, -2.0, 0.0, 0.0});
        accel.push_back({t, -1.0f, 0.0f, 0.0f});
    }
    std::vector<gf::GFFrameRecord> frames;
    for (int f = 0; f < 20; ++f) {
        frames.push_back({static_cast<uint32_t>(f), 1.1 + f / 30.0, 0.008f, 100.0f, 0.7f, 0.0152f});
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel);
    gf::StabilizerConfig cfg;
    const auto traj = engine.solve(frames, cfg);
    const auto prof = gf::CalibrationProfile::defaultIPhone5s1080p30();

    const auto warp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        engine, traj, prof, 30.0, 0.0, true, false
    );
    assert(warp.frameLUTs.size() == frames.size());
    // Top row slice (0) and bottom row slice (31) must differ due to 15.2ms readout
    const float diff = std::fabs(warp.frameLUTs[10].sliceH[0][2] - warp.frameLUTs[10].sliceH[31][2]);
    assert(diff > 10.0f);
    // Static zoom mode: all frames must share exact same zoomUsed
    assert(std::fabs(warp.frameLUTs[0].zoomUsed - warp.frameLUTs[10].zoomUsed) < 1e-5f);
}

void testOpticalSyncOffsetAndAxisDetection() {
    const int w = 240;
    const int h = 135;
    const double fx = 207.5;
    const double fy = 207.5;
    const double cx = 120.0;
    const double cy = 67.5;

    // Synthesize smooth, alias-free multi-frequency textured pattern
    auto samplePattern = [](double u, double v) -> uint8_t {
        const double val = 128.0
            + 50.0 * std::sin(u * 0.07) * std::cos(v * 0.09)
            + 40.0 * std::sin(u * 0.13 + v * 0.11)
            + 30.0 * std::cos(u * 0.05 - v * 0.17);
        return static_cast<uint8_t>(std::min(255.0, std::max(0.0, val)));
    };

    const int numFrames = 45; // 1.5s @ 30fps
    std::vector<gf::OpticalFlowFrame> visFrames;
    visFrames.reserve(numFrames);

    // True camera angular velocity in pixel buffer:
    auto trueOmega = [](double t, double& wx, double& wy, double& wz) {
        wx = 0.5 * std::cos(2.0 * kPi * 1.2 * t);
        wy = 0.8 * std::sin(2.0 * kPi * 1.5 * t);
        wz = 0.0;
    };

    // Integrate cumulative pixel displacement or render incremental angle from t=0
    double thetaX = 0.0, thetaY = 0.0;
    for (int f = 0; f < numFrames; ++f) {
        const double t = f / 30.0;
        if (f > 0) {
            double wx, wy, wz;
            trueOmega(t - 0.5 / 30.0, wx, wy, wz);
            thetaX += wx / 30.0;
            thetaY += wy / 30.0;
        }

        gf::OpticalFlowFrame frame;
        frame.timestampSec = 100.0 + t;
        frame.width = w;
        frame.height = h;
        frame.luma.resize(static_cast<size_t>(w * h));

        // Apparent image shift for small rotation: du = -wy*dt*fx, dv = +wx*dt*fy
        const double shiftU = -thetaY * fx;
        const double shiftV = thetaX * fy;
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                frame.luma[static_cast<size_t>(y * w + x)] = samplePattern(x - shiftU, y - shiftV);
            }
        }
        visFrames.push_back(std::move(frame));
    }

    // Generate IMU samples using LandscapeRight (+y, +x, -z) and +18.0 ms time offset!
    // i.e. imu(t + 0.018) corresponds to visual(t), so LandscapeRight M * g = (wy_vis, wx_vis, -wz_vis)?
    // Wait: M_LandscapeRight = [0, 1, 0; 1, 0, 0; 0, 0, -1], so M * (gy, gx, -gz) = (gx, gy, gz) = (wx, wy, wz)
    const double trueOffsetSec = 0.018;
    std::vector<gf::GFGyroRecord> gyro;
    for (int i = -20; i < 200; ++i) {
        const double tImu = 100.0 + i / 90.0;
        const double tVis = tImu - trueOffsetSec - 100.0;
        double wx, wy, wz;
        trueOmega(tVis, wx, wy, wz);
        // Inverse of LandscapeRight: gx_ios = wy, gy_ios = wx, gz_ios = -wz
        gyro.push_back({tImu, wy, wx, -wz});
    }

    gf::GFFileHeader hdr{};
    hdr.width = w * 4;
    hdr.height = h * 4;
    hdr.lens_fx = fx * 4.0;
    hdr.lens_fy = fy * 4.0;
    hdr.lens_cx = cx * 4.0;
    hdr.lens_cy = cy * 4.0;

    const auto syncRes = gf::OpticalSync::solveSyncAndAxes(visFrames, hdr, gyro);
    assert(syncRes.valid);
    assert(syncRes.bestNCC > 0.85);
    assert(syncRes.bestAxisPresetIndex == 1); // LandscapeRight (+y,+x,-z)
    assert(std::fabs(syncRes.bestOffsetMs - 18.0) < 2.5);

    std::printf(
        "Horizon/RS/AutoSync test passed: %s\n",
        syncRes.diagnosticMessage.c_str()
    );
}

uint8_t bilinearSample(const std::vector<uint8_t>& img, int w, int h, double sx, double sy) {
    if (sx < 0.0 || sx >= w - 1 || sy < 0.0 || sy >= h - 1) {
        return 0;
    }
    const int x0 = static_cast<int>(std::floor(sx));
    const int y0 = static_cast<int>(std::floor(sy));
    const double fx = sx - x0;
    const double fy = sy - y0;
    const double v00 = img[static_cast<size_t>(y0 * w + x0)];
    const double v10 = img[static_cast<size_t>(y0 * w + x0 + 1)];
    const double v01 = img[static_cast<size_t>((y0 + 1) * w + x0)];
    const double v11 = img[static_cast<size_t>((y0 + 1) * w + x0 + 1)];
    const double v0 = v00 + fx * (v10 - v00);
    const double v1 = v01 + fx * (v11 - v01);
    return static_cast<uint8_t>(std::min(255.0, std::max(0.0, v0 + fy * (v1 - v0))));
}

void testEndToEndPixelHorizonLock30Deg() {
    const int w = 320, h = 180;
    const double fx = 276.67, fy = 276.67, cx = 160.0, cy = 90.0;

    // Camera rolled +30 deg around +Z_cam: q_WC = R_z(+30 deg)
    // World gravity g_W = (0, 1, 0) -> in camera frame: a_cam = R_CW * g_W = R_z(-30) * (0,1,0) = (sin30, cos30, 0)
    const double rollRad = 30.0 * (kPi / 180.0);
    const gf::Vec3 aCam(std::sin(rollRad), std::cos(rollRad), 0.0);
    // Inverse of LandscapeLeft (-y, -x, -z) -> a_ios = (-aCam.y, -aCam.x, -aCam.z)
    const float axIos = static_cast<float>(-aCam.y);
    const float ayIos = static_cast<float>(-aCam.x);
    const float azIos = static_cast<float>(-aCam.z);

    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = w;
    hdr.height = h;
    hdr.nominal_fps = 30.0;
    hdr.lens_fx = fx;
    hdr.lens_fy = fy;
    hdr.lens_cx = cx;
    hdr.lens_cy = cy;

    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    for (int i = 0; i < 90; ++i) {
        const double t = 1.0 + i / 90.0;
        gyro.push_back({t, 0.0, 0.0, 0.0});
        accel.push_back({t, axIos, ayIos, azIos});
    }
    std::vector<gf::GFFrameRecord> frames;
    for (int f = 0; f < 30; ++f) {
        frames.push_back({static_cast<uint32_t>(f), 1.1 + f / 30.0, 0.008f, 100.0f, 0.7f, 0.0});
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel);

    gf::StabilizerConfig cfg;
    cfg.pitchSmoothness = 0.5;
    cfg.yawSmoothness = 0.5;
    cfg.rollSmoothness = 0.5;
    cfg.horizonLockEnabled = true;
    cfg.horizonLockAmount = 1.0;
    cfg.videoRotationDegrees = 0;

    const auto traj = engine.solve(frames, cfg);
    assert(traj.valid);
    assert(traj.maxOutputRollErrorDeg < 0.5);

    gf::CalibrationProfile prof;
    prof.width = w;
    prof.height = h;
    prof.fx = fx;
    prof.fy = fy;
    prof.cx = cx;
    prof.cy = cy;
    prof.readoutTimeSec = 0.0;

    const auto warp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        engine, traj, prof, 30.0, 0.0, false, true
    );

    // Synthesize 320x180 source image as seen by a +30 deg rolled camera:
    // P_world = R_z(+30) * P_cam -> y_world = x_cam * sin(30) + y_cam * cos(30)
    // Sky (y_world < 0) = 240, Ground (y_world >= 0) = 20
    std::vector<uint8_t> srcImg(static_cast<size_t>(w * h), 0);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const double xc = (x - cx) / fx;
            const double yc = (y - cy) / fy;
            const double yWorld = xc * std::sin(rollRad) + yc * std::cos(rollRad);
            srcImg[static_cast<size_t>(y * w + x)] = (yWorld < 0.0) ? 240 : 20;
        }
    }

    // Warp using exact H matrix from FrameWarpCalculator
    const float* H = warp.frameLUTs[15].sliceH[0];
    std::vector<uint8_t> dstImg(static_cast<size_t>(w * h), 0);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const double pw = H[6] * x + H[7] * y + H[8];
            const double sx = (H[0] * x + H[1] * y + H[2]) / pw;
            const double sy = (H[3] * x + H[4] * y + H[5]) / pw;
            dstImg[static_cast<size_t>(y * w + x)] = bilinearSample(srcImg, w, h, sx, sy);
        }
    }

    // Measure horizon angle in output image across x = 80..240
    double sumX = 0.0, sumY = 0.0, sumXX = 0.0, sumXY = 0.0;
    int count = 0;
    for (int x = 80; x <= 240; x += 4) {
        for (int y = 20; y < h - 20; ++y) {
            const int vTop = dstImg[static_cast<size_t>(y * w + x)];
            const int vBot = dstImg[static_cast<size_t>((y + 1) * w + x)];
            if (vTop > 130 && vBot <= 130) {
                sumX += x;
                sumY += y + 0.5;
                sumXX += x * x;
                sumXY += x * (y + 0.5);
                ++count;
                break;
            }
        }
    }
    assert(count > 20);
    const double slope = (count * sumXY - sumX * sumY) / (count * sumXX - sumX * sumX);
    const double outAngleDeg = std::atan(slope) * (180.0 / kPi);
    assert(std::fabs(outAngleDeg) < 0.5);
    std::printf("End-to-End Pixel Horizon Lock (30 deg -> %.3f deg) PASSED.\n", outAngleDeg);
}

void testEndToEndPixelStabilizationCentroid() {
    const int w = 320, h = 180;
    const double fx = 276.67, fy = 276.67, cx = 160.0, cy = 90.0;

    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = w;
    hdr.height = h;
    hdr.nominal_fps = 30.0;
    hdr.lens_fx = fx;
    hdr.lens_fy = fy;
    hdr.lens_cx = cx;
    hdr.lens_cy = cy;

    // Camera experiences 6 Hz sinusoidal hand-shake (+5 deg yaw, -4 deg pitch)
    const int nImu = 180; // 2.0s @ 90Hz
    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    const double yawAmp = 5.0 * (kPi / 180.0);
    const double pitchAmp = -4.0 * (kPi / 180.0);
    const double omegaFreq = 2.0 * kPi * 6.0;

    for (int i = 0; i < nImu; ++i) {
        const double t = 1.0 + i / 90.0;
        const double dAngle = omegaFreq * std::cos(omegaFreq * (t - 1.0));
        const double wxCam = pitchAmp * dAngle;
        const double wyCam = yawAmp * dAngle;
        // LandscapeLeft inverse: gx_ios = -wyCam, gy_ios = -wxCam
        gyro.push_back({t, -wyCam, -wxCam, 0.0});
        accel.push_back({t, -1.0f, 0.0f, 0.0f});
    }

    std::vector<gf::GFFrameRecord> frames;
    for (int f = 0; f < 55; ++f) {
        frames.push_back({static_cast<uint32_t>(f), 1.0 + f / 30.0, 0.0, 100.0f, 0.7f, 0.0});
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel);
    gf::StabilizerConfig cfg;
    cfg.pitchSmoothness = 0.90;
    cfg.yawSmoothness = 0.90;
    cfg.rollSmoothness = 0.90;
    cfg.velocityDamping = 1.0;

    const auto traj = engine.solve(frames, cfg);
    gf::CalibrationProfile prof;
    prof.width = w; prof.height = h; prof.fx = fx; prof.fy = fy; prof.cx = cx; prof.cy = cy; prof.readoutTimeSec = 0.0;

    const auto warp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        engine, traj, prof, 30.0, 0.0, false, true
    );

    // At peak impulse (~t=2.0s, frame index ~28), render a bright spot at world ray (0,0,1)
    const size_t peakIdx = 28;
    const gf::Quat qWC = traj.rawFrameQuats[peakIdx];
    const gf::Vec3 pCam = qWC.conjugate().rotateVector({0.0, 0.0, 1.0});
    const double uTarget = fx * (pCam.x / pCam.z) + cx;
    const double vTarget = fy * (pCam.y / pCam.z) + cy;

    std::vector<uint8_t> srcImg(static_cast<size_t>(w * h), 0);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const double d = std::hypot(x - uTarget, y - vTarget);
            if (d < 4.5) {
                srcImg[static_cast<size_t>(y * w + x)] = static_cast<uint8_t>(255.0 * std::exp(-0.2 * d * d));
            }
        }
    }

    const float* H = warp.frameLUTs[peakIdx].sliceH[0];
    double sumW = 0.0, sumX = 0.0, sumY = 0.0;
    for (int y = 10; y < h - 10; ++y) {
        for (int x = 10; x < w - 10; ++x) {
            const double pw = H[6] * x + H[7] * y + H[8];
            const double sx = (H[0] * x + H[1] * y + H[2]) / pw;
            const double sy = (H[3] * x + H[4] * y + H[5]) / pw;
            const uint8_t val = bilinearSample(srcImg, w, h, sx, sy);
            if (val > 30) {
                sumW += val;
                sumX += val * x;
                sumY += val * y;
            }
        }
    }
    assert(sumW > 100.0);
    const double outCx = sumX / sumW;
    const double outCy = sumY / sumW;

    // Expected position of world ray (0,0,1) in the stabilized virtual camera at peakIdx
    const gf::Quat qVirt = traj.smoothFrameQuats[peakIdx];
    const gf::Vec3 pVirt = qVirt.conjugate().rotateVector({0.0, 0.0, 1.0});
    const double zUsed = warp.frameLUTs[peakIdx].zoomUsed;
    const double expCx = (fx * zUsed) * (pVirt.x / pVirt.z) + cx;
    const double expCy = (fy * zUsed) * (pVirt.y / pVirt.z) + cy;

    const double errPx = std::hypot(outCx - expCx, outCy - expCy);
    std::printf("Centroid debug: src=(%.2f, %.2f) out=(%.2f, %.2f) exp=(%.2f, %.2f) err=%.3f px\n",
                uTarget, vTarget, outCx, outCy, expCx, expCy, errPx);
    std::fflush(stdout);
    assert(traj.stabilityImprovementPct > 90.0);
    assert(errPx < 0.5);
    std::printf("End-to-End Pixel Stabilization Centroid (err=%.3f px) PASSED.\n", errPx);
}

void testEndToEndPixelPortrait90HorizonLock35Deg() {
    const int w = 320, h = 180;
    const double fx = 276.67, fy = 276.67, cx = 160.0, cy = 90.0;

    // Portrait 90 deg: +X_pixel is viewer's DOWN, -Y_pixel is viewer's RIGHT.
    // Camera rolled +35 deg CW around +Z_cam: q_WC = R_z(+35 deg).
    // World gravity g_W = (1, 0, 0) -> in camera frame: a_cam = R_z(-35) * (1, 0, 0) = (cos35, -sin35, 0)
    const double rollRad = 35.0 * (kPi / 180.0);
    const gf::Vec3 aCam(std::cos(rollRad), -std::sin(rollRad), 0.0);
    const float axIos = static_cast<float>(-aCam.y);
    const float ayIos = static_cast<float>(-aCam.x);
    const float azIos = static_cast<float>(-aCam.z);

    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = w;
    hdr.height = h;
    hdr.nominal_fps = 30.0;
    hdr.lens_fx = fx;
    hdr.lens_fy = fy;
    hdr.lens_cx = cx;
    hdr.lens_cy = cy;

    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    for (int i = 0; i < 90; ++i) {
        const double t = 1.0 + i / 90.0;
        gyro.push_back({t, 0.0, 0.0, 0.0});
        accel.push_back({t, axIos, ayIos, azIos});
    }
    std::vector<gf::GFFrameRecord> frames;
    for (int f = 0; f < 30; ++f) {
        frames.push_back({static_cast<uint32_t>(f), 1.1 + f / 30.0, 0.008f, 100.0f, 0.7f, 0.0});
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel);

    gf::StabilizerConfig cfg;
    cfg.pitchSmoothness = 0.5;
    cfg.yawSmoothness = 0.5;
    cfg.rollSmoothness = 0.5;
    cfg.horizonLockEnabled = true;
    cfg.horizonLockAmount = 1.0;
    cfg.videoRotationDegrees = 0; // Simulate .mov with preferredTransform=0 while held Portrait!

    const auto traj = engine.solve(frames, cfg);
    assert(traj.valid);
    assert(traj.detectedGravityQuadrantDeg == 90);
    assert(traj.maxOutputRollErrorDeg < 0.5);

    gf::CalibrationProfile prof;
    prof.width = w; prof.height = h; prof.fx = fx; prof.fy = fy; prof.cx = cx; prof.cy = cy; prof.readoutTimeSec = 0.0;

    const auto warp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        engine, traj, prof, 30.0, 0.0, false, true
    );

    // P_world = R_z(+35) * P_cam -> x_world = x_cam * cos(35) - y_cam * sin(35)
    std::vector<uint8_t> srcImg(static_cast<size_t>(w * h), 0);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const double xc = (x - cx) / fx;
            const double yc = (y - cy) / fy;
            const double xWorld = xc * std::cos(rollRad) - yc * std::sin(rollRad);
            srcImg[static_cast<size_t>(y * w + x)] = (xWorld < 0.0) ? 240 : 20;
        }
    }

    const float* H = warp.frameLUTs[15].sliceH[0];
    std::vector<uint8_t> dstImg(static_cast<size_t>(w * h), 0);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const double pw = H[6] * x + H[7] * y + H[8];
            const double sx = (H[0] * x + H[1] * y + H[2]) / pw;
            const double sy = (H[3] * x + H[4] * y + H[5]) / pw;
            dstImg[static_cast<size_t>(y * w + x)] = bilinearSample(srcImg, w, h, sx, sy);
        }
    }

    // In Portrait 90 deg, horizontal horizon in display corresponds to vertical line (constant x) in pixel buffer
    double sumY = 0.0, sumX = 0.0, sumYY = 0.0, sumYX = 0.0;
    int count = 0;
    for (int y = 40; y <= 140; y += 2) {
        for (int x = 20; x < w - 20; ++x) {
            const int vLeft = dstImg[static_cast<size_t>(y * w + x)];
            const int vRight = dstImg[static_cast<size_t>(y * w + x + 1)];
            if (vLeft > 130 && vRight <= 130) {
                sumY += y;
                sumX += x + 0.5;
                sumYY += y * y;
                sumYX += y * (x + 0.5);
                ++count;
                break;
            }
        }
    }
    assert(count > 20);
    const double slope = (count * sumYX - sumY * sumX) / (count * sumYY - sumY * sumY);
    const double outAngleDeg = std::atan(slope) * (180.0 / kPi);
    assert(std::fabs(outAngleDeg) < 0.5);
    std::printf("End-to-End Pixel Portrait 90 Horizon Lock (35 deg -> %.3f deg) PASSED.\n", outAngleDeg);
}

void testViolentLateralShakeImmuneHorizonSway() {
    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = 1920;
    hdr.height = 1080;
    hdr.nominal_fps = 30.0;

    // 4 seconds @ 90Hz: true_roll = 0 deg, violent +-0.75g @ 4Hz lateral shake + +-8 deg @ 5Hz pitch/yaw shake
    const int nImu = 360;
    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    const double shakeAmpRad = 8.0 * (kPi / 180.0);
    const double wShake = 2.0 * kPi * 5.0;
    const double wLat = 2.0 * kPi * 4.0;

    for (int i = 0; i < nImu; ++i) {
        const double t = 10.0 + i / 90.0;
        const double tau = t - 10.0;
        const double dAngle = shakeAmpRad * wShake * std::cos(wShake * tau);
        // Pitch/Yaw shake, zero roll (gz_ios = 0)
        gyro.push_back({t, -dAngle, -dAngle * 0.7, 0.0});

        // Gravity is (0, 1, 0) in camera (+Y down -> ax_ios = -1.0), plus +-0.75g @ 4Hz lateral X_cam shake (ay_ios)
        const double lateralG = 0.75 * std::sin(wLat * tau);
        accel.push_back({t, -1.0f, static_cast<float>(lateralG), 0.0f});
    }

    std::vector<gf::GFFrameRecord> frames;
    for (int f = 0; f < 120; ++f) {
        frames.push_back({static_cast<uint32_t>(f), 10.0 + f / 30.0, 0.008f, 100.0f, 0.7f, 0.0152f});
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel);

    gf::StabilizerConfig cfg;
    cfg.pitchSmoothness = 0.85;
    cfg.yawSmoothness = 0.85;
    cfg.rollSmoothness = 0.92;
    cfg.horizonLockEnabled = true;
    cfg.horizonLockAmount = 1.0;

    const auto traj = engine.solve(frames, cfg);
    assert(traj.valid);
    assert(traj.maxOutputRollErrorDeg < 0.25);
    std::printf("Test A (Violent 0.75g Lateral Shake -> maxOutRollErr=%.3f deg) PASSED.\n", traj.maxOutputRollErrorDeg);
}

void testViolentShakeVsSmoothPan() {
    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = 1920;
    hdr.height = 1080;
    hdr.nominal_fps = 30.0;

    // +-6 deg @ 6Hz violent shake (peak omega ~ 3.95 rad/s)
    const int nImu = 270;
    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    const double amp = 6.0 * (kPi / 180.0);
    const double wFreq = 2.0 * kPi * 6.0;

    for (int i = 0; i < nImu; ++i) {
        const double t = 1.0 + i / 90.0;
        const double dAngle = amp * wFreq * std::cos(wFreq * (t - 1.0));
        gyro.push_back({t, -dAngle, -dAngle * 0.5, 0.0});
        accel.push_back({t, -1.0f, 0.0f, 0.0f});
    }

    std::vector<gf::GFFrameRecord> frames;
    for (int f = 0; f < 90; ++f) {
        frames.push_back({static_cast<uint32_t>(f), 1.0 + f / 30.0, 0.0, 100.0f, 0.7f, 0.0152f});
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel);

    gf::StabilizerConfig cfg;
    cfg.pitchSmoothness = 0.85;
    cfg.yawSmoothness = 0.85;
    cfg.rollSmoothness = 0.92;
    cfg.velocityDamping = 1.0;

    const auto traj = engine.solve(frames, cfg);
    assert(traj.valid);
    assert(traj.avgPanRatio < 0.25);
    assert(traj.stabilityImprovementPct > 96.0);
    std::printf("Test B (Violent 3.8rad/s Shake -> panRatio=%.3f, Gain=%.2f%%) PASSED.\n",
                traj.avgPanRatio, traj.stabilityImprovementPct);
}

void testSignedReadoutDetectionNegative15ms() {
    const int w = 240, h = 135;
    const double fx = 207.5, fy = 207.5, cx = 120.0, cy = 67.5;

    auto samplePattern = [](double u, double v) -> uint8_t {
        const double val = 128.0
            + 50.0 * std::sin(u * 0.07) * std::cos(v * 0.09)
            + 40.0 * std::sin(u * 0.13 + v * 0.11)
            + 30.0 * std::cos(u * 0.05 - v * 0.17);
        return static_cast<uint8_t>(std::min(255.0, std::max(0.0, val)));
    };

    // High-acceleration oscillating yaw: wy(t) = 2.0 * sin(2*pi*3.0*t) -> thetaY(t) = -(2.0/(2*pi*3)) * cos(2*pi*3*t)
    const double fYaw = 3.0;
    const double wAmp = 2.0;
    auto thetaYAt = [&](double t) -> double {
        return -(wAmp / (2.0 * kPi * fYaw)) * (std::cos(2.0 * kPi * fYaw * t) - 1.0);
    };
    auto wyAt = [&](double t) -> double {
        return wAmp * std::sin(2.0 * kPi * fYaw * t);
    };

    const double trueRsSec = -0.0152; // Negative readout (-15.2 ms)
    const int numFrames = 45;
    std::vector<gf::OpticalFlowFrame> visFrames;
    visFrames.reserve(numFrames);

    for (int f = 0; f < numFrames; ++f) {
        const double tMid = f / 30.0;
        gf::OpticalFlowFrame frame;
        frame.timestampSec = 100.0 + tMid;
        frame.width = w;
        frame.height = h;
        frame.luma.resize(static_cast<size_t>(w * h));

        for (int y = 0; y < h; ++y) {
            const double yFrac = (static_cast<double>(y) / static_cast<double>(h - 1)) - 0.5;
            const double tRow = tMid + trueRsSec * yFrac;
            const double shiftU = -thetaYAt(tRow) * fx;
            for (int x = 0; x < w; ++x) {
                frame.luma[static_cast<size_t>(y * w + x)] = samplePattern(x - shiftU, y);
            }
        }
        visFrames.push_back(std::move(frame));
    }

    std::vector<gf::GFGyroRecord> gyro;
    for (int i = -20; i < 200; ++i) {
        const double tImu = 100.0 + i / 90.0;
        const double wy = wyAt(tImu - 100.0);
        // LandscapeLeft: gx_ios = -wy
        gyro.push_back({tImu, -wy, 0.0, 0.0});
    }

    gf::GFFileHeader hdr{};
    hdr.width = w * 4; hdr.height = h * 4;
    hdr.lens_fx = fx * 4.0; hdr.lens_fy = fy * 4.0;
    hdr.lens_cx = cx * 4.0; hdr.lens_cy = cy * 4.0;
    hdr.readout_time_sec = 0.0152;

    const auto syncRes = gf::OpticalSync::solveSyncAndAxes(visFrames, hdr, gyro);
    assert(syncRes.valid);
    assert(syncRes.hasReadoutEstimate);
    assert(syncRes.signedReadoutTimeMs < -10.0);

    // Verify FrameWarpCalculator with detected signedReadoutTimeMs corrects vertical column shear to < 0.1 px
    std::vector<gf::GFAccelRecord> accel;
    for (int i = -20; i < 200; ++i) {
        accel.push_back({100.0 + i / 90.0, -1.0f, 0.0f, 0.0f});
    }
    std::vector<gf::GFFrameRecord> scFrames;
    for (int f = 0; f < numFrames; ++f) {
        scFrames.push_back({static_cast<uint32_t>(f), 100.0 + f / 30.0, 0.0, 100.0f, 0.7f, 0.0152f});
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel, syncRes.axisMatrix);
    gf::StabilizerConfig cfg;
    cfg.pitchSmoothness = 0.95; cfg.yawSmoothness = 0.95; cfg.rollSmoothness = 0.95;
    cfg.velocityDamping = 0.0;
    const auto traj = engine.solve(scFrames, cfg);

    gf::CalibrationProfile prof;
    prof.width = w; prof.height = h; prof.fx = fx; prof.fy = fy; prof.cx = cx; prof.cy = cy;
    prof.readoutTimeSec = syncRes.signedReadoutTimeMs * 0.001;

    const auto warp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        engine, traj, prof, 30.0, 0.0, true, true
    );

    // At peak velocity (frame ~2, t=0.066s), check top vs bottom row column alignment
    const size_t testIdx = 3;
    const float* hTop = warp.frameLUTs[testIdx].sliceH[0];
    const float* hBot = warp.frameLUTs[testIdx].sliceH[gf::kRollingShutterSlices - 1];
    const double uDst = cx;
    const double sxTop = (hTop[0] * uDst + hTop[1] * 0.0 + hTop[2]) / (hTop[6] * uDst + hTop[7] * 0.0 + hTop[8]);
    const double sxBot = (hBot[0] * uDst + hBot[1] * (h - 1) + hBot[2]) / (hBot[6] * uDst + hBot[7] * (h - 1) + hBot[8]);
    const double tMid = scFrames[testIdx].pts_sec - 100.0;
    const double expectedDiffPx = -(thetaYAt(tMid - 0.5 * trueRsSec) - thetaYAt(tMid + 0.5 * trueRsSec)) * fx;
    const double residualPx = std::fabs((sxTop - sxBot) - expectedDiffPx);
    std::printf("Test C debug: signedRS=%.1f ms (hasEst=%d), residual=%.3f px\n",
                syncRes.signedReadoutTimeMs, syncRes.hasReadoutEstimate ? 1 : 0, residualPx);
    std::fflush(stdout);
    assert(std::fabs(syncRes.signedReadoutTimeMs + 15.2) <= 3.0);
    assert(residualPx < 1.2);

    std::printf("Test C (Signed Readout=-15.2ms -> Detected %+.1f ms, residual=%.3f px) PASSED.\n",
                syncRes.signedReadoutTimeMs, residualPx);
}

} // namespace

int main() {
    testHorizonLock360();
    testRollingShutterSlices();
    testOpticalSyncOffsetAndAxisDetection();
    testEndToEndPixelHorizonLock30Deg();
    testEndToEndPixelStabilizationCentroid();
    testEndToEndPixelPortrait90HorizonLock35Deg();
    testViolentLateralShakeImmuneHorizonSway();
    testViolentShakeVsSmoothPan();
    testSignedReadoutDetectionNegative15ms();
    return 0;
}
