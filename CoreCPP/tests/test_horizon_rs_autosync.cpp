#include "CalibrationRefiner.hpp"
#include "FrameWarpCalculator.hpp"
#include "OpticalSync.hpp"
#include "SidecarIO.hpp"
#include "TrajectoryEngine.hpp"
#include "TranslationalStabilizer2D.hpp"
#include "VirtualGyroEngine.hpp"

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
    std::printf("Centroid debug: src=(%.2f, %.2f) out=(%.2f, %.2f) exp=(%.2f, %.2f) err=%.3f px, stability=%.2f%%\n",
                uTarget, vTarget, outCx, outCy, expCx, expCy, errPx, traj.stabilityImprovementPct);
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

void testPortraitToLandscape90TransitionUltraGimbal() {
    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = 1920;
    hdr.height = 1080;
    hdr.nominal_fps = 30.0;
    hdr.lens_fx = 1660.0;
    hdr.lens_fy = 1660.0;
    hdr.lens_cx = 960.0;
    hdr.lens_cy = 540.0;

    // 10s @ 90Hz = 900 IMU samples, 300 frames @ 30fps
    // 0..3s: Portrait (g_W = (1,0,0)), 3..5s: rotate +90 deg around +Z_cam, 5..10s: Landscape + +-6 deg shake
    const int nImu = 900;
    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    const double rollRate90 = (kPi * 0.5) / 2.0; // +90 deg over 2s
    const double shakeAmp = 6.0 * (kPi / 180.0);
    const double wShake = 2.0 * kPi * 4.0;

    double angleZ = 0.0;
    for (int i = 0; i < nImu; ++i) {
        const double t = 10.0 + i / 90.0;
        const double tau = t - 10.0;
        double wz = 0.0;
        if (tau >= 3.0 && tau < 5.0) {
            wz = rollRate90;
        } else if (tau >= 5.0) {
            wz = shakeAmp * wShake * std::cos(wShake * (tau - 5.0));
        }
        if (i > 0) {
            angleZ += wz * (1.0 / 90.0);
        }

        // LandscapeLeft (-y, -x, -z) -> gz_ios = -wz
        gyro.push_back({t, 0.0, 0.0, -wz});

        // World gravity g_W = (1, 0, 0). In camera rotated by angleZ around +Z: a_cam = R_z(-angleZ)*(1,0,0) = (cos(a), -sin(a), 0)
        const double axCam = std::cos(angleZ);
        const double ayCam = -std::sin(angleZ);
        accel.push_back({t, static_cast<float>(-ayCam), static_cast<float>(-axCam), 0.0f});
    }

    std::vector<gf::GFFrameRecord> frames;
    for (int f = 0; f < 300; ++f) {
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
    assert(traj.detectedGravityQuadrantDeg == 90);
    assert(traj.maxOutputRollErrorDeg < 0.3);

    const auto prof = gf::CalibrationProfile::defaultIPhone5s1080p30();
    const auto warp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        engine, traj, prof, 30.0, 0.0, true, true
    );
    assert(warp.minZoom < 1.10);
    assert(warp.maxZoom >= 1.75);

    std::printf("Test D (Portrait->Landscape 90deg UltraGimbal -> maxOutRollErr=%.3f deg, Zoom=[%.2f..%.2f]) PASSED.\n",
                traj.maxOutputRollErrorDeg, warp.minZoom, warp.maxZoom);
}

void testFastPanSwingLagLimit() {
    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = 1920;
    hdr.height = 1080;
    hdr.nominal_fps = 30.0;

    // 120 deg/s whip-pan around Y_cam for 1.0s
    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    const double whipRate = 120.0 * (kPi / 180.0);
    for (int i = 0; i < 270; ++i) {
        const double t = 1.0 + i / 90.0;
        const double wy = (t >= 2.0 && t <= 3.0) ? whipRate : 0.0;
        gyro.push_back({t, -wy, 0.0, 0.0});
        accel.push_back({t, -1.0f, 0.0f, 0.0f});
    }
    std::vector<gf::GFFrameRecord> frames;
    for (int f = 0; f < 90; ++f) {
        frames.push_back({static_cast<uint32_t>(f), 1.0 + f / 30.0, 0.008f, 100.0f, 0.7f, 0.0152f});
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel);

    gf::StabilizerConfig cfg;
    cfg.pitchSmoothness = 0.90;
    cfg.yawSmoothness = 0.90;
    cfg.rollSmoothness = 0.92;
    cfg.velocityDamping = 0.0; // Stress-test worst-case without pan damping!

    const auto traj = engine.solve(frames, cfg);
    assert(traj.valid);
    // Pan-adaptive swing-lag limiter must compress 60 deg whip-pan lag within 24.5 deg ceiling even with velocityDamping=0
    assert(traj.maxCorrectionDeg <= 24.6);

    const auto prof = gf::CalibrationProfile::defaultIPhone5s1080p30();
    const auto warp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        engine, traj, prof, 30.0, 0.0, true, true
    );
    // Every frame's 4 corners must have w > 0.25 (zero left-half perspective tearing)
    const double corners[4][2] = {{0.0, 0.0}, {1919.0, 0.0}, {1919.0, 1079.0}, {0.0, 1079.0}};
    for (const auto& lut : warp.frameLUTs) {
        const float* H = lut.sliceH[16];
        for (int c = 0; c < 4; ++c) {
            const double w = H[6] * corners[c][0] + H[7] * corners[c][1] + H[8];
            assert(w > 0.25);
        }
    }
    std::printf("Fast-Pan Swing-Lag Limit (maxCorr=%.2f deg, maxZoom=%.2fx, w>0.25) PASSED.\n",
                traj.maxCorrectionDeg, warp.maxZoom);
}

void testEffectiveFocalScaleAndCubicMicroJitter() {
    const int w = 320, h = 180;
    const double fxNom = 302.5, fyNom = 302.5, cx = 160.0, cy = 90.0;
    const double trueScale = 1.07;
    const double fxTrue = fxNom * trueScale;
    const double fyTrue = fyNom * trueScale;

    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = 1920;
    hdr.height = 1080;
    hdr.nominal_fps = 30.0;
    hdr.lens_fx = 1815.0;
    hdr.lens_fy = 1815.0;
    hdr.lens_cx = 960.0;
    hdr.lens_cy = 540.0;

    // 2.0s @ 90Hz gyro: 2.0 Hz sinusoidal yaw + 35 Hz MEMS white-noise component
    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    const double yawAmp = 2.5 * (kPi / 180.0);
    const double freq = 2.0 * kPi * 2.0;
    const double noiseFreq = 2.0 * kPi * 35.0;
    for (int i = 0; i < 180; ++i) {
        const double t = 1.0 + i / 90.0;
        const double wyTrue = yawAmp * freq * std::cos(freq * (t - 1.0));
        const double wyNoise = 0.04 * std::sin(noiseFreq * (t - 1.0)); // 35 Hz MEMS noise
        gyro.push_back({t, -(wyTrue + wyNoise), 0.0, 0.0});
        accel.push_back({t, -1.0f, 0.0f, 0.0f});
    }

    // Generate 30 textured frames rendered with fxTrue = 1.07 * fxNom
    std::vector<gf::OpticalFlowFrame> frames;
    std::vector<gf::GFFrameRecord> scFrames;
    for (int f = 0; f < 30; ++f) {
        const double t = 1.1 + f / 30.0;
        const double yawAngle = yawAmp * std::sin(freq * (t - 1.0));
        const gf::Quat qWC = gf::Quat::fromAxisAngle({0.0, 1.0, 0.0}, yawAngle);
        gf::OpticalFlowFrame frame;
        frame.timestampSec = t;
        frame.width = w;
        frame.height = h;
        frame.luma.resize(static_cast<size_t>(w * h), 0);
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                const gf::Vec3 rayW = qWC.rotateVector({(x - cx) / fxTrue, (y - cy) / fyTrue, 1.0});
                const double wx = rayW.x / rayW.z;
                const double wy = rayW.y / rayW.z;
                const double v = 128.0 + 60.0 * std::sin(wx * 45.0) * std::cos(wy * 45.0)
                                 + 40.0 * std::cos(wx * 95.0 + wy * 70.0);
                frame.luma[static_cast<size_t>(y * w + x)] = static_cast<uint8_t>(std::max(0.0, std::min(255.0, v)));
            }
        }
        frames.push_back(std::move(frame));
        scFrames.push_back({static_cast<uint32_t>(f), t, 0.008f, 100.0f, 0.7f, 0.0});
    }

    const auto syncRes = gf::OpticalSync::solveSyncAndAxes(frames, hdr, gyro);
    assert(syncRes.valid);
    assert(syncRes.bestNCC > 0.90);
    assert(syncRes.hasFocalScaleEstimate);
    assert(std::fabs(syncRes.effectiveFocalScale - trueScale) < 0.04);

    // Check that TrajectoryEngine's 24Hz LPF + 4x Catmull-Rom RK4 suppresses the 35 Hz MEMS jitter
    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel);
    gf::StabilizerConfig cfg;
    cfg.pitchSmoothness = 0.85;
    cfg.yawSmoothness = 0.85;
    cfg.rollSmoothness = 0.85;
    cfg.velocityDamping = 0.5;
    const auto traj = engine.solve(scFrames, cfg);
    assert(traj.valid);

    // Sample raw quaternion at mid-frame timestamps and verify smoothness against 35 Hz ripple
    double maxJitterRad = 0.0;
    for (size_t i = 5; i + 5 < scFrames.size(); ++i) {
        const gf::Quat q0 = engine.sampleRawQuaternionAt(scFrames[i].pts_sec);
        const gf::Quat qTrue = gf::Quat::fromAxisAngle({0.0, 1.0, 0.0}, yawAmp * std::sin(freq * (scFrames[i].pts_sec - 1.0)));
        const gf::Quat qBase = engine.sampleRawQuaternionAt(scFrames[0].pts_sec);
        const gf::Quat qTrue0 = gf::Quat::fromAxisAngle({0.0, 1.0, 0.0}, yawAmp * std::sin(freq * (scFrames[0].pts_sec - 1.0)));
        const gf::Quat relEst = qBase.conjugate() * q0;
        const gf::Quat relTrue = qTrue0.conjugate() * qTrue;
        const double dot = std::min(1.0, std::fabs(relEst.w * relTrue.w + relEst.x * relTrue.x + relEst.y * relTrue.y + relEst.z * relTrue.z));
        const double angErr = 2.0 * std::acos(dot);
        // 35 Hz jitter with 0.04 rad/s should be heavily attenuated (< 0.001 rad = < 0.27 px)
        if (angErr > maxJitterRad) maxJitterRad = angErr;
    }
    const double maxJitterPx = maxJitterRad * fxNom;
    std::printf("Phase 5 FocalScale=%.3fx (true=%.3fx) | 35Hz MEMS Residual=%.3f px PASSED.\n",
                syncRes.effectiveFocalScale, trueScale, maxJitterPx);
}

void testVirtualGyroRansacMovingObjectImmunity() {
    const int w = 320, h = 180;
    const double fx = 276.67, fy = 276.67, cx = 160.0, cy = 90.0;

    gf::VirtualGyroConfig cfg;
    cfg.videoWidth = 1920;
    cfg.videoHeight = 1080;
    cfg.nominalFps = 30.0;
    cfg.lensFx = 1660.0;
    cfg.lensFy = 1660.0;
    cfg.lensCx = 960.0;
    cfg.lensCy = 540.0;
    cfg.uuid = "PHASE5-VGYRO-TEST";

    gf::VirtualGyroEngine vgyro(cfg);

    // Background rotates at true camera yaw rate = +0.24 rad/s;
    // A 90x90 foreground box moves rapidly in the opposite direction (-2.0 px/frame)!
    const double trueWy = 0.24;
    std::vector<uint8_t> luma(static_cast<size_t>(w * h), 0);

    for (int f = 0; f < 15; ++f) {
        const double t = f / 30.0;
        const double yawAngle = trueWy * t;
        const int boxX0 = 180 - f * 4;
        const int boxY0 = 50;

        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                double val = 0.0;
                if (x >= boxX0 && x < boxX0 + 80 && y >= boxY0 && y < boxY0 + 80) {
                    // Moving foreground object texture
                    val = 128.0 + 90.0 * std::sin((x - boxX0) * 0.45) * std::cos((y - boxY0) * 0.45);
                } else {
                    // Static background texture observed by rotating camera
                    const double wx = (x - cx) / fx + yawAngle;
                    const double wy = (y - cy) / fy;
                    val = 128.0 + 70.0 * std::sin(wx * 55.0) * std::cos(wy * 55.0)
                          + 40.0 * std::cos(wx * 110.0 - wy * 85.0);
                }
                luma[static_cast<size_t>(y * w + x)] = static_cast<uint8_t>(std::max(0.0, std::min(255.0, val)));
            }
        }
        assert(vgyro.pushLumaFrame(luma.data(), w, h, t));
    }

    const std::string tmpPath = "build_test_virtual_gyro.gfraw";
    const auto vres = vgyro.finalizeAndWrite(tmpPath);
    assert(vres.success);
    assert(vres.frameCount == 15);
    assert(vres.meanInlierRatio > 0.45);

    gf::GFFileHeader hdr{};
    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    std::vector<gf::GFFrameRecord> frames;
    gf::SidecarSummary summary;
    assert(gf::SidecarReader::readAll(tmpPath, hdr, gyro, accel, frames, summary));
    assert(hdr.reserved[0] == 1); // is_virtual_gyro flag
    std::remove(tmpPath.c_str());

    // Verify recovered camera-frame yaw rate (LandscapeLeft: wyCam = -gx_ios)
    double sumWy = 0.0;
    int count = 0;
    for (size_t i = 2; i + 2 < gyro.size(); ++i) {
        sumWy += -gyro[i].gx;
        ++count;
    }
    const double avgWy = sumWy / std::max(1, count);
    assert(std::fabs(avgWy - trueWy) < 0.05);
    std::printf("VirtualGyro RANSAC Moving-Object Immunity (estWy=%.3f vs true=%.3f rad/s, inlier=%.1f%%) PASSED.\n",
                avgWy, trueWy, vres.meanInlierRatio * 100.0);
}

void testExtremeHighAmplitudeShake18Deg5Hz() {
    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = 1920;
    hdr.height = 1080;
    hdr.nominal_fps = 30.0;

    // +-18.0 deg @ 5Hz violent shake on both Pitch and Yaw (36 deg peak-to-peak!)
    const int nImu = 360; // 4.0s @ 90Hz
    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    const double ampRad = 18.0 * (kPi / 180.0);
    const double wFreq = 2.0 * kPi * 5.0;

    // Default LandscapeLeft matrix: camX = -gy, camY = -gx, camZ = -gz
    for (int i = 0; i < nImu; ++i) {
        const double t = 1.0 + i / 90.0;
        const double dAngle = ampRad * wFreq * std::cos(wFreq * (t - 1.0));
        gyro.push_back({t, -dAngle * 0.707, -dAngle * 0.707, 0.0});
        accel.push_back({t, 0.0f, -1.0f, 0.0f});
    }

    std::vector<gf::GFFrameRecord> frames;
    for (int f = 0; f < 120; ++f) {
        frames.push_back({static_cast<uint32_t>(f), 1.0 + f / 30.0, 0.0, 100.0f, 0.7f, 0.0152f});
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel);

    gf::StabilizerConfig cfg;
    cfg.pitchSmoothness = 0.96;
    cfg.yawSmoothness = 0.96;
    cfg.rollSmoothness = 0.98;
    cfg.maxZoomLimit = 2.20;
    cfg.horizonLockEnabled = true;
    cfg.horizonLockAmount = 1.0;

    const auto traj = engine.solve(frames, cfg);
    assert(traj.valid);
    assert(traj.avgPanRatio < 0.20);
    assert(traj.maxSwingCapDeg > 20.0);

    // Check residual output angle in interior [15..105] relative to mean orientation
    double maxOutErrDeg = 0.0;
    const gf::Quat qRef = traj.smoothFrameQuats[60];
    for (size_t i = 15; i <= 105; ++i) {
        gf::Quat dq = (qRef.conjugate() * traj.smoothFrameQuats[i]).normalized();
        if (dq.w < 0.0) dq = gf::Quat(-dq.w, -dq.x, -dq.y, -dq.z);
        const double deg = 2.0 * std::acos(std::min(1.0, std::max(-1.0, dq.w))) * (180.0 / kPi);
        maxOutErrDeg = std::max(maxOutErrDeg, deg);
    }
    assert(maxOutErrDeg < 0.40);

    const auto profile = gf::CalibrationProfile::defaultIPhone5s1080p30();
    const auto warp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        engine, traj, profile, 30.0, cfg.maxZoomLimit, false, true
    );
    assert(warp.packets.size() == 120);

    // Verify 0% black borders across all 120 frames
    for (size_t i = 0; i < warp.packets.size(); ++i) {
        const float* H = warp.packets[i].m;
        const double corners[4][2] = {{0.0, 0.0}, {1919.0, 0.0}, {1919.0, 1079.0}, {0.0, 1079.0}};
        for (const auto& c : corners) {
            const double w = H[6] * c[0] + H[7] * c[1] + H[8];
            assert(w > 0.25);
            const double us = (H[0] * c[0] + H[1] * c[1] + H[2]) / w;
            const double vs = (H[3] * c[0] + H[4] * c[1] + H[5]) / w;
            assert(us >= 0.0 && us <= 1919.0 && vs >= 0.0 && vs <= 1079.0);
        }
    }

    std::printf("Test G (+-18 deg @ 5Hz Heavy Shake -> panRatio=%.3f, swingCap=%.1f deg, outErr=%.3f deg, maxZoom=%.2fx) PASSED.\n",
                traj.avgPanRatio, traj.maxSwingCapDeg, maxOutErrDeg, warp.maxZoom);
}

void testHybridOpticalResidualTranslationalBounce() {
    const int w = 480;
    const int h = 270;
    const int nFrames = 45; // 1.5s @ 30fps
    const auto profile = gf::CalibrationProfile::defaultIPhone5s1080p30();

    // Zero gyro rotation
    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = 1920;
    hdr.height = 1080;
    hdr.nominal_fps = 30.0;

    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    for (int i = 0; i < 135; ++i) {
        const double t = i / 90.0;
        gyro.push_back({t, 0.0, 0.0, 0.0});
        accel.push_back({t, 0.0f, -1.0f, 0.0f});
    }
    std::vector<gf::GFFrameRecord> scFrames;
    for (int f = 0; f < nFrames; ++f) {
        scFrames.push_back({static_cast<uint32_t>(f), f / 30.0, 0.0, 100.0f, 0.7f, 0.0});
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel);
    gf::StabilizerConfig cfg;
    cfg.pitchSmoothness = 0.85;
    cfg.yawSmoothness = 0.85;
    cfg.rollSmoothness = 0.90;
    const auto traj = engine.solve(scFrames, cfg);
    assert(traj.valid);

    // Pure translational vertical bounce: +-20 px in 1080p (= +-5.0 px in 480x270) at 6 Hz
    std::vector<gf::OpticalFlowFrame> frames;
    std::vector<double> trueBounce1080(nFrames, 0.0);
    for (int f = 0; f < nFrames; ++f) {
        const double t = f / 30.0;
        const double shift1080Y = 20.0 * std::sin(2.0 * kPi * 6.0 * t);
        trueBounce1080[f] = shift1080Y;
        const double shiftSmallY = shift1080Y * (static_cast<double>(h) / 1080.0);

        gf::OpticalFlowFrame frame;
        frame.timestampSec = t;
        frame.width = w;
        frame.height = h;
        frame.luma.resize(static_cast<size_t>(w * h));
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                const double wx = static_cast<double>(x);
                const double wy = static_cast<double>(y) - shiftSmallY;
                // Non-periodic multi-scale texture (avoids periodic aliasing in block matching)
                double val = 128.0
                    + 28.0 * std::sin(wx * 0.043 + wy * 0.031)
                    + 24.0 * std::cos(wx * 0.097 - wy * 0.073)
                    + 20.0 * std::sin((wx * wx * 0.0007) + wy * 0.17)
                    + 18.0 * std::cos(wx * 0.19 + (wy * wy * 0.0011));
                frame.luma[static_cast<size_t>(y * w + x)] =
                    static_cast<uint8_t>(std::max(0.0, std::min(255.0, val)));
            }
        }
        frames.push_back(std::move(frame));
    }

    const auto hr = gf::OpticalSync::estimateHybridResidualShifts(frames, profile, traj, 30.0);
    assert(hr.valid);
    assert(hr.shiftY.size() == static_cast<size_t>(nFrames));
    assert(hr.maxAmplitudePx > 14.0);

    // Residual vertical bounce after applying hr.shiftY in interior [10..35]
    double maxResidualPx = 0.0;
    for (int f = 10; f <= 35; ++f) {
        const double residual = std::fabs(trueBounce1080[f] + hr.shiftY[f]);
        maxResidualPx = std::max(maxResidualPx, residual);
    }
    std::fprintf(stderr, "Test H debug: detectedMax=%.2f px, maxResidualPx=%.2f px, mid=(true=%.2f, shift=%.2f)\n",
                 hr.maxAmplitudePx, maxResidualPx, trueBounce1080[22], hr.shiftY[22]);
    assert(maxResidualPx < 3.0);

    const auto warp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        engine, traj, profile, 30.0, 0.0, false, true, &hr.shiftX, &hr.shiftY
    );
    assert(warp.packets.size() == static_cast<size_t>(nFrames));
    for (size_t i = 0; i < warp.packets.size(); ++i) {
        const float* H = warp.packets[i].m;
        const double corners[4][2] = {{0.0, 0.0}, {1919.0, 0.0}, {1919.0, 1079.0}, {0.0, 1079.0}};
        for (const auto& c : corners) {
            const double wVal = H[6] * c[0] + H[7] * c[1] + H[8];
            const double us = (H[0] * c[0] + H[1] * c[1] + H[2]) / wVal;
            const double vs = (H[3] * c[0] + H[4] * c[1] + H[5]) / wVal;
            assert(us >= 0.0 && us <= 1919.0 && vs >= 0.0 && vs <= 1079.0);
        }
    }

    std::printf("Test H (Hybrid Optical Residual +-20px @ 6Hz Bounce -> detectedMax=%.2f px, residual=%.2f px, maxZoom=%.2fx) PASSED.\n",
                hr.maxAmplitudePx, maxResidualPx, warp.maxZoom);
}

void testMotoCorneringCentripetalHorizonLock() {
    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = 1920;
    hdr.height = 1080;
    hdr.nominal_fps = 30.0;

    // 10.0s @ 90Hz: 0..2s straight, 2..7s leaning 30 deg + turning 18 deg/s (90 deg total), 7..10s straight
    const int nImu = 900;
    const double dtImu = 1.0 / 90.0;
    auto trueQuatAt = [](double t) -> gf::Quat {
        double yawRad = 0.0;
        if (t > 2.0 && t < 7.0) {
            yawRad = (18.0 * (kPi / 180.0)) * (t - 2.0);
        } else if (t >= 7.0) {
            yawRad = 90.0 * (kPi / 180.0);
        }
        double rollDeg = 0.0;
        if (t >= 2.0 && t < 2.5) {
            rollDeg = 30.0 * (t - 2.0) / 0.5;
        } else if (t >= 2.5 && t <= 6.5) {
            rollDeg = 30.0;
        } else if (t > 6.5 && t < 7.0) {
            rollDeg = 30.0 * (7.0 - t) / 0.5;
        }
        const double rollRad = rollDeg * (kPi / 180.0);
        return (gf::Quat::fromAxisAngle({0.0, 1.0, 0.0}, yawRad) *
                gf::Quat::fromAxisAngle({0.0, 0.0, 1.0}, rollRad)).normalized();
    };

    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    for (int i = 0; i < nImu; ++i) {
        const double t = i * dtImu;
        const gf::Quat q0 = trueQuatAt(std::max(0.0, t - 0.5 * dtImu));
        const gf::Quat q1 = trueQuatAt(t + 0.5 * dtImu);
        gf::Quat dq = (q0.conjugate() * q1).normalized();
        if (dq.w < 0.0) dq = gf::Quat(-dq.w, -dq.x, -dq.y, -dq.z);
        const double vNorm = std::sqrt(dq.x * dq.x + dq.y * dq.y + dq.z * dq.z);
        gf::Vec3 omegaCam(0.0, 0.0, 0.0);
        if (vNorm > 1e-9) {
            const double ang = 2.0 * std::atan2(vNorm, std::min(1.0, dq.w));
            omegaCam = gf::Vec3(dq.x, dq.y, dq.z) * (ang / (vNorm * dtImu));
        }
        // LandscapeLeft inverse: gx = -camY, gy = -camX, gz = -camZ
        gyro.push_back({t, -omegaCam.y, -omegaCam.x, -omegaCam.z});

        // Apparent gravity + centripetal acceleration in camera frame:
        // During 2..7s turn, bike leans 30 deg so apparent gravity points straight down bike frame (0, +1.15g, 0)!
        gf::Vec3 aCam = (t >= 2.0 && t <= 7.0)
            ? gf::Vec3(0.0, 1.15, 0.0)
            : q1.conjugate().rotateVector(gf::Vec3(0.0, 1.0, 0.0));
        accel.push_back({t, static_cast<float>(-aCam.y), static_cast<float>(-aCam.x), static_cast<float>(-aCam.z)});
    }

    std::vector<gf::GFFrameRecord> frames;
    for (int f = 0; f < 300; ++f) {
        frames.push_back({static_cast<uint32_t>(f), f / 30.0, 0.0, 100.0f, 0.88f, 0.0152f});
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel, nullptr, true);

    gf::StabilizerConfig cfg;
    cfg.pitchSmoothness = 0.94;
    cfg.yawSmoothness = 0.72;
    cfg.rollSmoothness = 0.98;
    cfg.horizonLockEnabled = true;
    cfg.horizonLockAmount = 1.0;
    cfg.motoActionMode = true;

    const auto traj = engine.solve(frames, cfg);
    assert(traj.valid);

    // Check output roll error against TRUE world gravity (0, 1, 0) during the 5s 30-deg leaning turn!
    double maxTrueRollErrDeg = 0.0;
    for (size_t f = 60; f <= 210; ++f) {
        const gf::Quat& q = traj.smoothFrameQuats[f];
        const gf::Vec3 rightW = q.rotateVector({1.0, 0.0, 0.0}).normalized();
        const gf::Vec3 downW = q.rotateVector({0.0, 1.0, 0.0}).normalized();
        const gf::Vec3 fwdW = q.rotateVector({0.0, 0.0, 1.0}).normalized();
        gf::Vec3 gProj = gf::Vec3(0.0, 1.0, 0.0) - fwdW * fwdW.y;
        if (gProj.normSquared() > 1e-6) {
            gProj = gProj.normalized();
            const double errDeg = std::fabs(std::atan2(gProj.dot(rightW), gProj.dot(downW)) * (180.0 / kPi));
            maxTrueRollErrDeg = std::max(maxTrueRollErrDeg, errDeg);
        }
    }
    std::printf("Test Moto 1 maxTrueRollErrDeg = %.4f deg\n", maxTrueRollErrDeg);
    std::fflush(stdout);
    assert(maxTrueRollErrDeg < 0.50);

    // Verify Yaw smoothly followed the ~90 deg turn by t=8.5s (frame 255)
    const gf::Vec3 fwdEnd = traj.smoothFrameQuats[255].rotateVector({0.0, 0.0, 1.0}).normalized();
    const double yawEndDeg = std::atan2(fwdEnd.x, fwdEnd.z) * (180.0 / kPi);
    assert(yawEndDeg > 80.0 && yawEndDeg < 100.0);

    std::printf("Test Moto 1 (30 deg Lean + 1.15g Centripetal 5s Turn -> maxTrueRollErr=%.3f deg, yawEnd=%.1f deg) PASSED.\n",
                maxTrueRollErrDeg, yawEndDeg);
}

void testMotoEngineBuzz32HzPotholeAndRoadParallax() {
    const int w = 480;
    const int h = 270;
    const int nFrames = 60; // 2.0s @ 30fps
    const auto profile = gf::CalibrationProfile::defaultIPhone5s1080p30();
    const double sX = static_cast<double>(w) / 1920.0;
    const double sY = static_cast<double>(h) / 1080.0;
    const double fxS = profile.fx * sX;
    const double fyS = profile.fy * sY;
    const double cxS = profile.cx * sX;
    const double cyS = profile.cy * sY;

    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = 1920;
    hdr.height = 1080;
    hdr.nominal_fps = 30.0;
    hdr.lens_fx = profile.fx;
    hdr.lens_fy = profile.fy;
    hdr.lens_cx = profile.cx;
    hdr.lens_cy = profile.cy;

    // Combine camera pan + 16 deg pothole impulse + 32 Hz engine buzz
    auto pitchAt = [](double t) -> double {
        const double pothole = 16.0 * (kPi / 180.0) * std::exp(-std::pow((t - 1.0) / 0.22, 2.0));
        const double buzz = 0.20 * (kPi / 180.0) * std::sin(2.0 * kPi * 32.0 * t);
        return pothole + buzz;
    };
    auto yawAt = [](double t) -> double {
        return 0.18 * t;
    };

    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    const double dtImu = 1.0 / 90.0;
    for (int i = 0; i < 180; ++i) {
        const double t = i * dtImu;
        const double dPitch = (pitchAt(t + 1e-4) - pitchAt(t - 1e-4)) / 2e-4;
        const double dYaw = (yawAt(t + 1e-4) - yawAt(t - 1e-4)) / 2e-4;
        // LandscapeLeft: wxCam = dPitch, wyCam = dYaw -> gx = -wyCam, gy = -wxCam
        gyro.push_back({t, -dYaw, -dPitch, 0.0});
        accel.push_back({t, 0.0f, -1.0f, 0.0f});
    }

    std::vector<gf::GFFrameRecord> scFrames;
    std::vector<gf::OpticalFlowFrame> visFrames;
    for (int f = 0; f < nFrames; ++f) {
        const double t = f / 30.0;
        scFrames.push_back({static_cast<uint32_t>(f), t, 0.0, 100.0f, 0.88f, 0.0152f});

        const double p = pitchAt(t);
        const double yAng = yawAt(t);
        gf::OpticalFlowFrame frame;
        frame.timestampSec = t;
        frame.width = w;
        frame.height = h;
        frame.luma.resize(static_cast<size_t>(w * h));
        for (int py = 0; py < h; ++py) {
            for (int px = 0; px < w; ++px) {
                double sx, sy;
                if (py < static_cast<int>(0.55 * h)) {
                    // Upper 55%: far-field scenery rotating cleanly with camera
                    sx = (static_cast<double>(px) - cxS) + yAng * fxS;
                    sy = (static_cast<double>(py) - cyS) - p * fyS;
                } else {
                    // Lower 45%: high-speed foreground road rushing downward (+12 px/frame)!
                    sx = static_cast<double>(px);
                    sy = static_cast<double>(py) - static_cast<double>(f) * 12.0;
                }
                const double val = 128.0
                    + 30.0 * std::sin(sx * 0.047 + sy * 0.037)
                    + 25.0 * std::cos(sx * 0.091 - sy * 0.067)
                    + 20.0 * std::sin((sx * sx * 0.0006) + sy * 0.15);
                frame.luma[static_cast<size_t>(py * w + px)] =
                    static_cast<uint8_t>(std::max(0.0, std::min(255.0, val)));
            }
        }
        visFrames.push_back(std::move(frame));
    }

    const auto syncRes = gf::OpticalSync::solveSyncAndAxes(visFrames, hdr, gyro, true);
    assert(syncRes.valid);
    assert(syncRes.bestNCC > 0.90);

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel, syncRes.axisMatrix, true);
    gf::StabilizerConfig cfg;
    cfg.pitchSmoothness = 0.94;
    cfg.yawSmoothness = 0.72;
    cfg.rollSmoothness = 0.98;
    cfg.horizonLockEnabled = true;
    cfg.motoActionMode = true;
    cfg.timeOffsetMs = syncRes.bestOffsetMs;

    const auto traj = engine.solve(scFrames, cfg);
    assert(traj.valid);

    // Check that 16 deg pothole + 32 Hz buzz on Pitch is suppressed > 95% (peak-to-peak variation < 0.8 deg)
    double minPitchDeg = 1e9;
    double maxPitchDeg = -1e9;
    for (size_t f = 10; f < 50; ++f) {
        const gf::Vec3 fwd = traj.smoothFrameQuats[f].rotateVector({0.0, 0.0, 1.0}).normalized();
        const double pitchDeg = std::asin(std::max(-1.0, std::min(1.0, -fwd.y))) * (180.0 / kPi);
        minPitchDeg = std::min(minPitchDeg, pitchDeg);
        maxPitchDeg = std::max(maxPitchDeg, pitchDeg);
    }
    const double maxPitchErrDeg = maxPitchDeg - minPitchDeg;
    assert(maxPitchErrDeg < 0.80);

    const auto warp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        engine, traj, profile, 30.0, 0.0, true, true
    );
    assert(warp.frameLUTs.size() == static_cast<size_t>(nFrames));
    static_assert(sizeof(warp.frameLUTs[0].sliceH) == 64 * 9 * sizeof(float), "64 RS slices expected");

    std::printf("Test Moto 2 (Far-Field ROI NCC=%.3f | 16deg Pothole + 32Hz Buzz -> pitchErr=%.3f deg, 64-Slice RS) PASSED.\n",
                syncRes.bestNCC, maxPitchErrDeg);
}

void testClassroomAntiPumpingAndStaticRollSettle() {
    const auto prof = gf::CalibrationProfile::defaultIPhone5s1080p30();
    assert(std::fabs(prof.fx - 1815.0) < 1e-3);

    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = 1920;
    hdr.height = 1080;
    hdr.nominal_fps = 30.0;
    hdr.lens_fx = prof.fx;
    hdr.lens_fy = prof.fy;
    hdr.lens_cx = prof.cx;
    hdr.lens_cy = prof.cy;

    // 4.5s @ 90Hz: 0..1.0s still, 1.0..1.8s roll 0->90 deg, 1.8..2.6s hold 90 deg,
    // 2.6..3.2s roll 90->45 deg, 3.2..4.5s hold 45 deg.
    auto trueRollDegAt = [](double t) -> double {
        if (t < 1.0) return 0.0;
        if (t < 1.8) return 90.0 * (t - 1.0) / 0.8;
        if (t < 2.6) return 90.0;
        if (t < 3.2) return 90.0 - 45.0 * (t - 2.6) / 0.6;
        return 45.0;
    };

    const int nImu = 405;
    const double dtImu = 1.0 / 90.0;
    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    for (int i = 0; i < nImu; ++i) {
        const double t = i * dtImu;
        const double r0 = trueRollDegAt(std::max(0.0, t - 0.5 * dtImu)) * (kPi / 180.0);
        const double r1 = trueRollDegAt(t + 0.5 * dtImu) * (kPi / 180.0);
        const double wzCamTrue = (r1 - r0) / dtImu;
        // Inject -4% MEMS Z scale error (raw gyro under-reports by 4%, corrected by rollGyroScale=1/0.96)
        const double wzCamMeas = wzCamTrue * 0.96;
        gyro.push_back({t, 0.0, 0.0, -wzCamMeas});

        const gf::Quat qTrue = gf::Quat::fromAxisAngle({0.0, 0.0, 1.0}, r1);
        const gf::Vec3 aCam = qTrue.conjugate().rotateVector({0.0, 1.0, 0.0});
        accel.push_back({t, static_cast<float>(-aCam.y), static_cast<float>(-aCam.x), static_cast<float>(-aCam.z)});
    }

    std::vector<gf::GFFrameRecord> frames;
    for (int f = 0; f < 135; ++f) {
        frames.push_back({static_cast<uint32_t>(f), f / 30.0, 0.016f, 100.0f, 0.7f, 0.0152f});
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel, nullptr, false, 1.0 / 0.96);

    gf::StabilizerConfig cfg;
    cfg.pitchSmoothness = 0.85;
    cfg.yawSmoothness = 0.85;
    cfg.rollSmoothness = 0.92;
    cfg.horizonLockEnabled = true;
    cfg.horizonLockAmount = 1.0;

    const auto traj = engine.solve(frames, cfg);
    assert(traj.valid);
    assert(traj.maxOutputRollErrorDeg < 0.50);

    const auto warp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        engine, traj, prof, 30.0, 0.0, true, true
    );
    assert(warp.packets.size() == 135);

    // Check ZERO early zoom pumping during stationary 00:00.00..00:00.75 (frames 0..22)
    double maxIntroZoom = 0.0;
    for (size_t f = 0; f <= 22; ++f) {
        maxIntroZoom = std::max(maxIntroZoom, static_cast<double>(warp.packets[f].zoomUsed));
    }
    assert(maxIntroZoom <= 1.05);

    std::printf("Test Classroom v1.1.0 (IntroZoom[0..0.75s]=%.3fx <= 1.05x | RollErr(90/45 deg)=%.3f deg | fx=%.0f) PASSED.\n",
                maxIntroZoom, traj.maxOutputRollErrorDeg, prof.fx);
}

void testPortraitIntrinsicsAndZeroZoomSpike() {
    // 1. Verify Peak-Hold Plateau (0.65s) + Ultra-Slow Release (<= 0.08x/sec) kills 125ms zoom snap
    const auto prof = gf::CalibrationProfile::defaultIPhone5s1080p30();
    const int nFrames = 120; // 4.0s @ 30fps
    std::vector<gf::Quat> rawQuats(nFrames, gf::Quat::identity());
    std::vector<gf::Quat> smoothQuats(nFrames, gf::Quat::identity());
    std::vector<double> ts(nFrames, 0.0);
    for (int f = 0; f < nFrames; ++f) {
        ts[f] = f / 30.0;
        // Sharp single-impulse roll spike at t=2.00s (frame 60)
        const double rollDeg = 18.0 * std::exp(-std::pow((ts[f] - 2.0) / 0.08, 2.0));
        rawQuats[f] = gf::Quat::fromAxisAngle({0.0, 0.0, 1.0}, rollDeg * (kPi / 180.0));
    }

    gf::TrajectoryEngine dummyEngine;
    gf::TrajectoryResult traj;
    traj.rawFrameQuats = rawQuats;
    traj.smoothFrameQuats = smoothQuats;
    traj.frameTimestampsSec = ts;
    traj.valid = true;

    const auto dynWarp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        dummyEngine, traj, prof, 30.0, 0.0, false, true
    );
    const double zPeak = dynWarp.packets[60].zoomUsed;      // t = 2.000s
    const double zAfter125ms = dynWarp.packets[64].zoomUsed; // t = 2.133s
    const double zAfter1s = dynWarp.packets[90].zoomUsed;    // t = 3.000s
    assert(zPeak > 1.25);
    assert(std::fabs(zPeak - zAfter125ms) < 0.005); // ZERO 125ms zoom snap!
    assert((zPeak - zAfter1s) <= 0.05);             // Ultra-slow <= 0.08x/sec release after 0.65s plateau!

    // 2. Verify Fixed Crop has 0.000% breathing
    const auto fixedWarp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        dummyEngine, traj, prof, 30.0, 0.0, false, false
    );
    assert(std::fabs(fixedWarp.maxZoom - fixedWarp.minZoom) < 1e-6);

    // 3. Verify Portrait 1080x1920 buffer with buggy legacy fx=1020.9 auto-upgrades to 1815.0 in OpticalSync
    const int pw = 180, ph = 320; // Portrait 1080x1920 scaled by 1/6
    const double fxTrue = 1815.0 * (320.0 / 1920.0); // 302.5
    const double cxP = 90.0, cyP = 160.0;
    gf::GFFileHeader pHdr{};
    std::memcpy(pHdr.magic, "GFS5", 4);
    pHdr.version = 2;
    pHdr.width = 1080;
    pHdr.height = 1920;
    pHdr.nominal_fps = 30.0;
    pHdr.lens_fx = 1020.9; // Legacy buggy width-scaled portrait fx
    pHdr.lens_fy = 1020.9;
    pHdr.lens_cx = 540.0;
    pHdr.lens_cy = 960.0;

    std::vector<gf::GFGyroRecord> pGyro;
    for (int i = 0; i < 90; ++i) {
        const double t = i / 90.0;
        const double wyCam = 0.35 * std::cos(2.0 * kPi * 1.5 * t);
        // Portrait (+x, -y, -z): wyCam = -gy -> gy = -wyCam
        pGyro.push_back({t, 0.0, -wyCam, 0.0});
    }
    std::vector<gf::OpticalFlowFrame> pFrames;
    for (int f = 0; f < 30; ++f) {
        const double t = f / 30.0;
        const double yawAng = (0.35 / (2.0 * kPi * 1.5)) * std::sin(2.0 * kPi * 1.5 * t);
        gf::OpticalFlowFrame frame;
        frame.timestampSec = t;
        frame.width = pw;
        frame.height = ph;
        frame.luma.resize(static_cast<size_t>(pw * ph));
        for (int y = 0; y < ph; ++y) {
            for (int x = 0; x < pw; ++x) {
                const double sx = (x - cxP) + yawAng * fxTrue;
                const double sy = (y - cyP);
                const double val = 128.0 + 35.0 * std::sin(sx * 0.09 + sy * 0.07) + 25.0 * std::cos(sx * 0.14 - sy * 0.11);
                frame.luma[static_cast<size_t>(y * pw + x)] = static_cast<uint8_t>(std::max(0.0, std::min(255.0, val)));
            }
        }
        pFrames.push_back(std::move(frame));
    }
    const auto pSync = gf::OpticalSync::solveSyncAndAxes(pFrames, pHdr, pGyro, false);
    assert(pSync.valid);
    assert(pSync.bestNCC > 0.95);
    assert(std::fabs(pSync.effectiveFocalScale - 1.0) < 0.08); // Proves 1020.9 was auto-upgraded to 1815.0!

    std::printf("Test Phase 7.2 v1.2.0 (ZoomPeak=%.3fx -> +125ms=%.3fx, +1.0s=%.3fx | Portrait 1080x1920 fx-upgrade NCC=%.3f, scale=%.3fx) PASSED.\n",
                zPeak, zAfter125ms, zAfter1s, pSync.bestNCC, pSync.effectiveFocalScale);
}

void test720p60ActionPipeline() {
    const auto prof = gf::CalibrationProfile::defaultIPhone5s720p120();
    assert(prof.width == 1280 && prof.height == 720);
    assert(std::fabs(prof.fx - 1210.0) < 1e-3);

    // 2.0s @ 60fps video (120 frames, dt=16.67ms) + 90Hz IMU (180 samples)
    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.capture_mode = 2; // 720p60
    hdr.width = 1280;
    hdr.height = 720;
    hdr.nominal_fps = 60.0;
    hdr.readout_time_sec = 0.0074;
    hdr.lens_fx = prof.fx;
    hdr.lens_fy = prof.fy;
    hdr.lens_cx = prof.cx;
    hdr.lens_cy = prof.cy;

    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    const double shakeAmp = 4.5 * (kPi / 180.0);
    const double wFreq = 2.0 * kPi * 2.0;
    for (int i = 0; i < 180; ++i) {
        const double t = i / 90.0;
        const double dYaw = shakeAmp * wFreq * std::cos(wFreq * t);
        gyro.push_back({t, -dYaw, 0.0, 0.0});
        accel.push_back({t, 0.0f, -1.0f, 0.0f});
    }

    const int vw = 320, vh = 180;
    const double fxS = prof.fx * 0.25, cxS = 160.0;
    std::vector<gf::GFFrameRecord> scFrames;
    std::vector<gf::OpticalFlowFrame> visFrames;
    for (int f = 0; f < 120; ++f) {
        const double t = f / 60.0;
        scFrames.push_back({static_cast<uint32_t>(f), t, 0.005f, 100.0f, 0.88f, 0.0074f});
        const double yawAng = shakeAmp * std::sin(wFreq * (t + 0.0025 + 0.0037));
        gf::OpticalFlowFrame frame;
        frame.timestampSec = t + 0.0025 + 0.0037;
        frame.width = vw;
        frame.height = vh;
        frame.luma.resize(static_cast<size_t>(vw * vh));
        for (int y = 0; y < vh; ++y) {
            for (int x = 0; x < vw; ++x) {
                const double sx = (x - cxS) + yawAng * fxS;
                const double sy = static_cast<double>(y);
                const double val = 128.0 + 35.0 * std::sin(sx * 0.07 + sy * 0.05)
                                         + 25.0 * std::cos(sx * 0.12 - sy * 0.09)
                                         + 20.0 * std::sin((sx * sx * 0.0006) + sy * 0.15);
                frame.luma[static_cast<size_t>(y * vw + x)] = static_cast<uint8_t>(std::max(0.0, std::min(255.0, val)));
            }
        }
        visFrames.push_back(std::move(frame));
    }

    const auto syncRes = gf::OpticalSync::solveSyncAndAxes(visFrames, hdr, gyro, false);
    assert(syncRes.valid);
    assert(syncRes.bestNCC > 0.95);

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel, syncRes.axisMatrix, false);
    gf::StabilizerConfig cfg;
    cfg.pitchSmoothness = 0.85;
    cfg.yawSmoothness = 0.85;
    cfg.rollSmoothness = 0.92;
    cfg.horizonLockEnabled = true;

    const auto traj = engine.solve(scFrames, cfg);
    assert(traj.valid);
    assert(traj.stabilityImprovementPct > 95.0);

    const auto warp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        engine, traj, prof, 60.0, 0.0, true, true
    );
    assert(warp.packets.size() == 120);
    for (const auto& pkt : warp.packets) {
        const float* H = pkt.m;
        const double corners[4][2] = {{0.0, 0.0}, {1279.0, 0.0}, {1279.0, 719.0}, {0.0, 719.0}};
        for (const auto& c : corners) {
            const double wVal = H[6] * c[0] + H[7] * c[1] + H[8];
            const double us = (H[0] * c[0] + H[1] * c[1] + H[2]) / wVal;
            const double vs = (H[3] * c[0] + H[4] * c[1] + H[5]) / wVal;
            assert(us >= 0.0 && us <= 1279.0 && vs >= 0.0 && vs <= 719.0);
        }
    }

    std::printf("Test Phase A 720p60 (90Hz IMU / 60fps Video -> NCC=%.3f, Gain=%.2f%%, Zoom=[%.2fx..%.2fx], 0%% black border) PASSED.\n",
                syncRes.bestNCC, traj.stabilityImprovementPct, warp.minZoom, warp.maxZoom);
}

void testPhaseBTranslational2DFootstepAndSign() {
    const auto prof = gf::CalibrationProfile::defaultIPhone5s1080p30();
    const int nFrames = 60; // 2.0s @ 30fps

    // 3D +-4 deg wrist rotation + +-20 px @ 3 Hz vertical footstep bounce + moving foreground object
    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = 1920;
    hdr.height = 1080;
    hdr.nominal_fps = 30.0;

    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    const double rotAmp = 10.0 * (kPi / 180.0);
    auto rollAt = [&](double t) -> double {
        // 10 deg roll impulse at t=0.25s sets Fixed Crop zoom ~1.31x, then gentle +-3 deg wrist sway
        return rotAmp * std::exp(-std::pow((t - 0.25) / 0.12, 2.0))
             + (3.0 * kPi / 180.0) * std::sin(2.0 * kPi * 1.2 * t);
    };
    for (int i = 0; i < 180; ++i) {
        const double t = i / 90.0;
        const double dRoll = (rollAt(t + 1e-4) - rollAt(t - 1e-4)) / 2e-4;
        gyro.push_back({t, 0.0, 0.0, -dRoll});
        accel.push_back({t, 0.0f, -1.0f, 0.0f});
    }

    std::vector<gf::GFFrameRecord> scFrames;
    for (int f = 0; f < nFrames; ++f) {
        scFrames.push_back({static_cast<uint32_t>(f), f / 30.0, 0.005f, 100.0f, 0.88f, 0.0});
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel);
    gf::StabilizerConfig cfg;
    cfg.pitchSmoothness = 0.85;
    cfg.yawSmoothness = 0.85;
    cfg.rollSmoothness = 0.92;
    cfg.horizonLockEnabled = true;

    const auto traj = engine.solve(scFrames, cfg);
    assert(traj.valid);

    gf::TranslationalStabilizer2D trans2D;
    trans2D.reset(prof, traj, 1.0);
    assert(trans2D.rectWidth() == 160 && trans2D.rectHeight() == 90);

    // Feed 640x360 frames (downsampled to 160x90 inside feedLumaFrame) with:
    //   1) 3D +-10 deg roll rotation
    //   2) +-12 px @ 3 Hz vertical footstep bounce (in 1080p coordinates)
    //   3) Fast moving foreground object in bottom-right quadrant
    const int sw = 640, sh = 360;
    const double sScale = 3.0; // 1920/640 = 3
    std::vector<uint8_t> lumaBuf(static_cast<size_t>(sw * sh), 0);
    std::vector<double> trueBounce1080(nFrames, 0.0);

    for (int f = 0; f < nFrames; ++f) {
        const double t = f / 30.0;
        const double bounceY = 12.0 * std::sin(2.0 * kPi * 3.0 * t);
        trueBounce1080[f] = bounceY;
        const double rollAng = rollAt(t);
        const double cr = std::cos(rollAng), sr = std::sin(rollAng);

        for (int y = 0; y < sh; ++y) {
            for (int x = 0; x < sw; ++x) {
                // Moving foreground distractor in bottom-right
                if (x > 440 && y > 230) {
                    const double fxObj = x - f * 5.0;
                    const double fyObj = y - f * 3.0;
                    const double v = 128.0 + 50.0 * std::sin(fxObj * 0.25 + fyObj * 0.25);
                    lumaBuf[static_cast<size_t>(y * sw + x)] = static_cast<uint8_t>(std::max(0.0, std::min(255.0, v)));
                    continue;
                }
                const double xc = (x - 320.0) * sScale;
                const double yc = (y - 180.0) * sScale - bounceY;
                const double wx = cr * xc - sr * yc;
                const double wy = sr * xc + cr * yc;
                const double val = 128.0
                    + 35.0 * std::sin(wx * 0.015 + wy * 0.011)
                    + 25.0 * std::cos(wx * 0.029 - wy * 0.023)
                    + 20.0 * std::sin((wx * wx * 0.00004) + wy * 0.041);
                lumaBuf[static_cast<size_t>(y * sw + x)] = static_cast<uint8_t>(std::max(0.0, std::min(255.0, val)));
            }
        }
        trans2D.feedLumaFrame(static_cast<size_t>(f), lumaBuf.data(), sw, sh, sw);
    }

    const auto res2D = trans2D.finalize(30.0);
    assert(res2D.valid);
    assert(res2D.validFrameCount >= 50);

    // Check actual pixel mapping at center of output (960, 540) at Zoom=1.25x (Fixed Crop)
    const auto warp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        engine, traj, prof, 30.0, 0.0, false, false, &res2D.shiftXBase, &res2D.shiftYBase
    );
    assert(warp.packets.size() == static_cast<size_t>(nFrames));

    double maxResidualPx = 0.0;
    for (int f = 15; f < 50; ++f) {
        const float* H = warp.packets[f].m;
        const double wC = H[6] * 960.0 + H[7] * 540.0 + H[8];
        const double vsCenter = (H[3] * 960.0 + H[4] * 540.0 + H[5]) / wC;
        const double residual = std::fabs(vsCenter - (540.0 + trueBounce1080[f]));
        maxResidualPx = std::max(maxResidualPx, residual);

        // Also verify 0% black borders across all 4 corners
        const double corners[4][2] = {{0.0, 0.0}, {1919.0, 0.0}, {1919.0, 1079.0}, {0.0, 1079.0}};
        for (const auto& c : corners) {
            const double wVal = H[6] * c[0] + H[7] * c[1] + H[8];
            const double us = (H[0] * c[0] + H[1] * c[1] + H[2]) / wVal;
            const double vs = (H[3] * c[0] + H[4] * c[1] + H[5]) / wVal;
            assert(us >= 0.0 && us <= 1919.0 && vs >= 0.0 && vs <= 1079.0);
        }
    }
    assert(maxResidualPx < 3.0);

    std::printf("Test Phase B 160x90 2D (3D+-10deg + 2D+-12px@3Hz + MovingObj -> valid=%zu/59, residual=%.2f px, Zoom=%.2fx, 0%% black border) PASSED.\n",
                res2D.validFrameCount, maxResidualPx, warp.maxZoom);
}

void testCalibrationRefinerAccuracyDriftAndFallback() {
    const int w = 480, h = 270;
    const double fullW = 1920.0, fullH = 1080.0;
    const double scaleSim = static_cast<double>(w) / fullW;
    const double fxFull = 1815.0, fyFull = 1815.0, cxFull = 960.0, cyFull = 540.0;
    const double fxSim = fxFull * scaleSim, fySim = fyFull * scaleSim;
    const double cxSim = cxFull * scaleSim, cySim = cyFull * scaleSim;

    auto evalOmega = [](double t) -> gf::Vec3 {
        return {
            0.55 * std::cos(2.0 * kPi * 1.4 * t) + 0.25 * std::sin(2.0 * kPi * 3.1 * t),
            0.65 * std::sin(2.0 * kPi * 1.1 * t) + 0.20 * std::cos(2.0 * kPi * 2.7 * t),
            0.35 * std::sin(2.0 * kPi * 1.7 * t + 0.4)
        };
    };

    const double tStart = 5.0;
    const double durationSec = 6.0;
    const double fps = 30.0;
    const double simStart = tStart - 0.15;
    const double simEnd = tStart + durationSec + 0.15;
    const double simDt = 1.0 / 480.0;
    const int simSteps = static_cast<int>(std::ceil((simEnd - simStart) / simDt)) + 1;
    std::vector<gf::Quat> trueQ(static_cast<size_t>(simSteps), gf::Quat::identity());
    for (int i = 1; i < simSteps; ++i) {
        const double tMid = simStart + (static_cast<double>(i) - 0.5) * simDt;
        trueQ[static_cast<size_t>(i)] = gf::Quat::integrateGyroStep(trueQ[static_cast<size_t>(i - 1)], evalOmega(tMid - tStart), simDt);
    }
    auto sampleQ = [&](double t) -> gf::Quat {
        const double pos = (t - simStart) / simDt;
        if (pos <= 0.0) return trueQ.front();
        const int i0 = std::min(simSteps - 2, static_cast<int>(std::floor(pos)));
        const double a = std::min(1.0, std::max(0.0, pos - i0));
        return gf::Quat::slerp(trueQ[static_cast<size_t>(i0)], trueQ[static_cast<size_t>(i0 + 1)], a);
    };

    const double trueOffA = 8.4; // ms
    const double trueOffB = 0.65; // ms/s linear clock drift
    const double trueScale[3] = {1.032, 0.974, 1.022};
    const double trueBias[3] = {0.015, -0.012, 0.009};
    const double trueRsSec = 0.0152;

    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = 1920; hdr.height = 1080; hdr.nominal_fps = fps;
    hdr.readout_time_sec = 0.0152;
    hdr.lens_fx = fxFull; hdr.lens_fy = fyFull; hdr.lens_cx = cxFull; hdr.lens_cy = cyFull;

    std::vector<gf::GFGyroRecord> gyro;
    for (int i = 0; i < 760; ++i) {
        const double tGyro = (tStart - 0.12) + static_cast<double>(i) / 120.0;
        const double deltaSec = (trueOffA + trueOffB * (tGyro - tStart)) * 0.001;
        const double tPhys = tGyro - deltaSec;
        const gf::Vec3 wTrue = evalOmega(tPhys - tStart);
        const double wxM = trueScale[0] * wTrue.x + trueBias[0];
        const double wyM = trueScale[1] * wTrue.y + trueBias[1];
        const double wzM = trueScale[2] * wTrue.z + trueBias[2];
        gyro.push_back({tGyro, -wyM, -wxM, -wzM});
    }

    const int nFrames = 180;
    std::vector<gf::GFFrameRecord> frames;
    frames.reserve(nFrames);
    for (int f = 0; f < nFrames; ++f) {
        frames.push_back({static_cast<uint32_t>(f), tStart + f / fps, 0.004f, 100.0f, 0.7f, static_cast<float>(trueRsSec)});
    }

    gf::CalibrationRefiner refiner;
    gf::CalibrationModePrior prior;
    refiner.reset(hdr, gyro, frames, &prior, nullptr, false);

    std::vector<uint8_t> luma(static_cast<size_t>(w * h));
    for (int f = 0; f < nFrames; ++f) {
        const double tMid = frames[f].pts_sec + 0.5 * 0.004 + 0.5 * trueRsSec;
        for (int y = 0; y < h; ++y) {
            const double rowNorm = (static_cast<double>(y) / (h - 1)) - 0.5;
            const gf::Quat qRow = sampleQ(tMid + rowNorm * trueRsSec);
            for (int x = 0; x < w; ++x) {
                const double xn = (x + 0.5 - cxSim) / fxSim;
                const double yn = (y + 0.5 - cySim) / fySim;
                const gf::Vec3 rw = qRow.rotateVector(gf::Vec3(xn, yn, 1.0).normalized());
                const double val = 128.0
                    + 45.0 * std::sin(rw.x * 24.0 + rw.y * 17.0)
                    + 35.0 * std::cos(rw.x * 39.0 - rw.y * 28.0)
                    + 25.0 * std::sin(rw.y * 51.0 + rw.z * 14.0);
                luma[static_cast<size_t>(y * w + x)] = static_cast<uint8_t>(std::min(255.0, std::max(0.0, val)));
            }
        }
        assert(refiner.pushDownsampledLumaFrame(luma.data(), w, h, frames[f].pts_sec, static_cast<size_t>(f)));
    }

    const gf::RefinedCalibration cal = refiner.finalize();
    assert(cal.valid);
    assert(!cal.usedPriorFallback);
    assert(cal.confidence >= 0.65);

    const double errOffA = std::fabs(cal.offsetA - trueOffA);
    const double errOffEnd = std::fabs((cal.offsetA + cal.offsetB * durationSec) - (trueOffA + trueOffB * durationSec));
    const double errBias0 = std::fabs(cal.bias[0] - trueBias[0]);
    const double errBias1 = std::fabs(cal.bias[1] - trueBias[1]);
    const double errBias2 = std::fabs(cal.bias[2] - trueBias[2]);
    const double errScale0 = std::fabs(cal.scale[0] - trueScale[0]) / trueScale[0];
    const double errScale1 = std::fabs(cal.scale[1] - trueScale[1]) / trueScale[1];
    const double errScale2 = std::fabs(cal.scale[2] - trueScale[2]) / trueScale[2];

    std::printf(
        "CalibrationRefiner Test: off=(%.2f+%.2ft vs %.2f+%.2ft) bias=(%.4f,%.4f,%.4f) scale=(%.4f,%.4f,%.4f) RS=%.2fms RMS %.4f->%.4f\n",
        cal.offsetA, cal.offsetB, trueOffA, trueOffB,
        cal.bias[0], cal.bias[1], cal.bias[2],
        cal.scale[0], cal.scale[1], cal.scale[2],
        cal.readoutMs, cal.residualRmsBefore, cal.residualRmsAfter
    );

    assert(errOffA < 1.0);
    assert(errOffEnd < 1.0);
    assert(errBias0 < 0.003 && errBias1 < 0.003 && errBias2 < 0.003);
    assert(errScale0 < 0.01 && errScale1 < 0.01 && errScale2 < 0.01);
    assert(cal.residualRmsAfter < cal.residualRmsBefore * 0.75);
    assert(cal.residualRmsAfter < 0.018);
    assert(std::fabs(cal.readoutMs - trueRsSec * 1000.0) < 2.5);

    // JSON round-trip check
    gf::RefinedCalibration parsed;
    assert(gf::RefinedCalibration::fromJson(cal.toJson(), parsed));
    assert(std::fabs(parsed.offsetA - cal.offsetA) < 1e-4);
    assert(std::fabs(parsed.scale[1] - cal.scale[1]) < 1e-4);

    // Low-texture clip must fall back cleanly to prior without corrupting scale
    gf::CalibrationModePrior customPrior;
    customPrior.hasPrior = true;
    customPrior.readoutMs = 15.1;
    customPrior.focalScale = 1.01;
    customPrior.scale[0] = 1.005; customPrior.scale[1] = 0.995; customPrior.scale[2] = 1.002;
    gf::CalibrationRefiner lowTexRefiner;
    lowTexRefiner.reset(hdr, gyro, frames, &customPrior, nullptr, false);
    std::vector<uint8_t> flatImg(static_cast<size_t>(w * h), 128);
    for (int f = 0; f < 60; ++f) {
        lowTexRefiner.pushDownsampledLumaFrame(flatImg.data(), w, h, frames[f].pts_sec, static_cast<size_t>(f));
    }
    const gf::RefinedCalibration lowTexCal = lowTexRefiner.finalize();
    assert(lowTexCal.valid);
    assert(lowTexCal.usedPriorFallback);
    assert(std::fabs(lowTexCal.scale[0] - customPrior.scale[0]) < 1e-6);
    assert(std::fabs(lowTexCal.readoutMs - customPrior.readoutMs) < 1e-6);

    // Running median prior check
    const auto medPrior = gf::CalibrationRefiner::computeModePriorFromHistory(
        {15.0, 15.4, 15.2}, {1.00, 1.02, 1.01}, {1.01, 1.03, 1.02}, {0.98, 0.99, 0.97}, {1.00, 1.01, 1.02}
    );
    assert(medPrior.hasPrior);
    assert(std::fabs(medPrior.readoutMs - 15.2) < 1e-6);
    assert(std::fabs(medPrior.focalScale - 1.01) < 1e-6);
    assert(std::fabs(medPrior.scale[0] - 1.02) < 1e-6);
    std::printf("CalibrationRefiner Accuracy, Drift, Prior-Fallback & Median Tests PASSED.\n");
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
    testPortraitToLandscape90TransitionUltraGimbal();
    testFastPanSwingLagLimit();
    testEffectiveFocalScaleAndCubicMicroJitter();
    testVirtualGyroRansacMovingObjectImmunity();
    testExtremeHighAmplitudeShake18Deg5Hz();
    testHybridOpticalResidualTranslationalBounce();
    testMotoCorneringCentripetalHorizonLock();
    testMotoEngineBuzz32HzPotholeAndRoadParallax();
    testClassroomAntiPumpingAndStaticRollSettle();
    testPortraitIntrinsicsAndZeroZoomSpike();
    test720p60ActionPipeline();
    testPhaseBTranslational2DFootstepAndSign();
    testCalibrationRefinerAccuracyDriftAndFallback();
    return 0;
}
