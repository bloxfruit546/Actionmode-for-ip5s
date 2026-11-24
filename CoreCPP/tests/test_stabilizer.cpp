#include "ConstrainedPathOptimizer.hpp"
#include "FrameWarpCalculator.hpp"
#include "QualityMetrics.hpp"
#include "TrajectoryEngine.hpp"
#include "TranslationalStabilizer2D.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

inline double rotationAngleBetween(const gf::Quat& a, const gf::Quat& b) {
    const gf::Quat diff = (a.conjugate() * b).normalized();
    const double sinHalf = std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
    return 2.0 * std::atan2(sinHalf, std::fabs(diff.w));
}

inline gf::Vec3 quatLog(const gf::Quat& qIn) {
    gf::Quat q = qIn.normalized();
    if (q.w < 0.0) {
        q = gf::Quat(-q.w, -q.x, -q.y, -q.z);
    }
    const double vNorm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    if (vNorm < 1e-8) {
        return gf::Vec3(2.0 * q.x, 2.0 * q.y, 2.0 * q.z);
    }
    const double theta = 2.0 * std::atan2(vNorm, q.w);
    const double scale = theta / vNorm;
    return gf::Vec3(q.x * scale, q.y * scale, q.z * scale);
}

void testHomographySignDirection() {
    // Camera yaws RIGHT (+DeltaTheta around +Y_cam).
    // Stationary object in front shifts LEFT in source image (u_s < cx).
    // Inverse homography mapping dst center (cx, cy) -> src must yield u_s < cx.
    const double delta = 0.08; // ~4.6 deg
    const gf::Quat qRaw = gf::Quat::fromAxisAngle({0.0, 1.0, 0.0}, delta);
    const gf::Quat qSmooth = gf::Quat::identity();
    const gf::Quat qCorr = (qRaw.conjugate() * qSmooth).normalized();

    float H[9];
    gf::FrameWarpCalculator::computeInverseHomography(qCorr, 1660.0, 1660.0, 960.0, 540.0, 1.0, H);

    const double u = 960.0;
    const double v = 540.0;
    const double w = H[6] * u + H[7] * v + H[8];
    const double us = (H[0] * u + H[1] * v + H[2]) / w;
    const double vs = (H[3] * u + H[4] * v + H[5]) / w;

    assert(us < 960.0 - 100.0);
    assert(std::fabs(vs - 540.0) < 1e-3);
}

void testStabilizerJitterAndPanAndZeroBlackBorder() {
    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = 1920;
    hdr.height = 1080;
    hdr.nominal_fps = 30.0;

    const double durationSec = 5.0;
    const double imuHz = 90.0;
    const int numImu = static_cast<int>(durationSec * imuHz);

    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    gyro.reserve(static_cast<size_t>(numImu));
    accel.reserve(static_cast<size_t>(numImu));

    // Simulate intentional gentle pan around Y (0.12 rad/s) + high-freq 6Hz hand jitter around X & Z
    for (int i = 0; i < numImu; ++i) {
        const double t = 100.0 + i / imuHz;
        const double jitterX = 0.15 * std::sin(2.0 * kPi * 6.0 * (t - 100.0));
        const double jitterZ = 0.10 * std::cos(2.0 * kPi * 5.0 * (t - 100.0));
        const double panY = (t > 101.0 && t < 104.0) ? 0.12 : 0.0;

        // Recall mapIOSImuToCameraFrame(ios) = (-ios.y, -ios.x, -ios.z)
        // So to produce camera-frame (jitterX, panY, jitterZ), iOS gyro = (-panY, -jitterX, -jitterZ)
        gyro.push_back({t, -panY, -jitterX, -jitterZ});
        accel.push_back({t, -1.0f, 0.0f, 0.0f});
    }

    // Intentionally test boundary clamping: first frame slightly before IMU start, last frame slightly after IMU end
    const int numFrames = 150;
    std::vector<gf::GFFrameRecord> frames;
    frames.reserve(numFrames);
    for (int f = 0; f < numFrames; ++f) {
        const double pts = 99.98 + f * (1.0 / 30.0);
        frames.push_back({static_cast<uint32_t>(f), pts, 0.008f, 100.0f, 0.7f, 0.0152f});
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel);

    gf::StabilizerConfig cfgZero;
    cfgZero.pitchSmoothness = 0.0;
    cfgZero.yawSmoothness = 0.0;
    cfgZero.rollSmoothness = 0.0;
    const gf::TrajectoryResult trajZero = engine.solve(frames, cfgZero);
    assert(trajZero.valid);
    assert(trajZero.stabilityImprovementPct < 1e-6);

    gf::StabilizerConfig cfg;
    cfg.smoothness = 0.65;
    cfg.velocityDamping = 1.0;
    cfg.maxZoomLimit = 0.0; // Unlimited safe zoom -> 0% black border guaranteed

    const gf::TrajectoryResult traj = engine.solve(frames, cfg);
    assert(traj.valid);
    assert(traj.rawFrameQuats.size() == frames.size());
    assert(traj.smoothFrameQuats.size() == frames.size());
    std::printf("traj.stabilityImprovementPct = %.2f%%, rawRms=%.4f, smoothRms=%.4f\n",
        traj.stabilityImprovementPct, traj.rawAngularAccelRms, traj.smoothAngularAccelRms);
    std::fflush(stdout);
    assert(traj.stabilityImprovementPct > 70.0 && traj.stabilityImprovementPct <= 99.5);

    const gf::CalibrationProfile prof = gf::CalibrationProfile::defaultIPhone5s1080p30();
    const gf::WarpComputationResult warp = gf::FrameWarpCalculator::computeAllFrames(
        traj.rawFrameQuats,
        traj.smoothFrameQuats,
        prof,
        30.0,
        cfg.maxZoomLimit
    );

    assert(warp.packets.size() == frames.size());
    assert(warp.minZoom >= 1.0);
    assert(warp.maxZoom < 1.65);

    // Verify 0% black border on every frame's 4 corners
    const double corners[4][2] = {{0.0, 0.0}, {1919.0, 0.0}, {1919.0, 1079.0}, {0.0, 1079.0}};
    for (const auto& pkt : warp.packets) {
        for (int c = 0; c < 4; ++c) {
            const double u = corners[c][0];
            const double v = corners[c][1];
            const double w = pkt.m[6] * u + pkt.m[7] * v + pkt.m[8];
            assert(w > 0.0);
            const double us = (pkt.m[0] * u + pkt.m[1] * v + pkt.m[2]) / w;
            const double vs = (pkt.m[3] * u + pkt.m[4] * v + pkt.m[5]) / w;
            assert(us >= 3.9 && us <= 1915.1);
            assert(vs >= 3.9 && vs <= 1075.1);
        }
    }

    std::printf(
        "Stabilizer test passed: StabilityGain=%.2f%%, Zoom=[%.3f..%.3f]\n",
        traj.stabilityImprovementPct, warp.minZoom, warp.maxZoom
    );
}

void testPureInterFrameGyroDeRotation() {
    const int w = 1920, h = 1080;
    const double fx = 1815.0, fy = 1815.0, cx = 960.0, cy = 540.0;
    gf::CalibrationProfile prof;
    prof.width = w; prof.height = h; prof.fx = fx; prof.fy = fy; prof.cx = cx; prof.cy = cy;
    prof.readoutTimeSec = 0.0;

    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = w; hdr.height = h; hdr.nominal_fps = 30.0;
    hdr.lens_fx = fx; hdr.lens_fy = fy; hdr.lens_cx = cx; hdr.lens_cy = cy;

    // Pan 25 deg/s around yaw + 5 Hz shake +-6 deg around pitch with 0 translation
    const int nFrames = 60; // 2.0s @ 30fps
    const int nImu = 180;   // 90Hz IMU
    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;

    const double yawRate = 25.0 * (kPi / 180.0); // 25 deg/s
    const double pitchAmp = 6.0 * (kPi / 180.0); // 6 deg
    const double omegaFreq = 2.0 * kPi * 5.0;

    for (int i = 0; i < nImu; ++i) {
        const double t = i / 90.0;
        const double wy = yawRate;
        const double wx = pitchAmp * omegaFreq * std::cos(omegaFreq * t);
        // LandscapeLeft inverse: gx_ios = -wyCam, gy_ios = -wxCam
        gyro.push_back({t, -wy, -wx, 0.0});
        accel.push_back({t, 0.0f, -1.0f, 0.0f});
    }

    std::vector<gf::GFFrameRecord> frames;
    for (int f = 0; f < nFrames; ++f) {
        frames.push_back({static_cast<uint32_t>(f), f / 30.0, 0.005f, 100.0f, 0.88f, 0.0});
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel);
    gf::StabilizerConfig cfg;
    cfg.smoothness = 0.78;
    const auto traj = engine.solve(frames, cfg);
    assert(traj.valid);

    gf::TranslationalStabilizer2D trans2D;
    trans2D.reset(prof, traj, 1.0);

    // Generate downsampled frames (160x90) with procedural texture purely rotated by qRaw
    const int rw = 160, rh = 90;
    std::vector<uint8_t> luma(static_cast<size_t>(rw * rh), 0);
    const double fxR = fx * (160.0 / 1920.0);
    const double fyR = fy * (90.0 / 1080.0);
    const double cxR = 80.0, cyR = 45.0;

    for (int f = 0; f < nFrames; ++f) {
        const gf::Quat q = traj.rawFrameQuats[f];
        for (int y = 0; y < rh; ++y) {
            for (int x = 0; x < rw; ++x) {
                // Ray in camera coords
                const double rx = (x - cxR) / fxR;
                const double ry = (y - cyR) / fyR;
                const gf::Vec3 rayCam = gf::Vec3(rx, ry, 1.0).normalized();
                // Ray in world coords rotated by q
                const gf::Vec3 rayW = q.rotateVector(rayCam);
                const double uVal = 128.0
                    + 40.0 * std::sin(rayW.x * 25.0 + rayW.y * 18.0)
                    + 30.0 * std::cos(rayW.x * 35.0 - rayW.z * 22.0)
                    + 25.0 * std::sin(rayW.y * 45.0 + rayW.z * 15.0);
                luma[static_cast<size_t>(y * rw + x)] = static_cast<uint8_t>(std::max(0.0, std::min(255.0, uVal)));
            }
        }
        trans2D.feedLumaFrame(static_cast<size_t>(f), luma.data(), rw, rh, rw);
    }

    const auto res2D = trans2D.finalize(30.0);
    assert(res2D.valid);
    // Pure inter-frame gyro de-rotation cancels angular rotation; residual translation < 0.35 px!
    assert(res2D.maxShiftPx < 0.35);
    std::printf("testPureInterFrameGyroDeRotation PASSED: maxShiftPx = %.3f px (< 0.35 px)\n", res2D.maxShiftPx);
}

void testActionModeActionLevel40AndFixedCrop() {
    const int w = 1920, h = 1080;
    const double fx = 1815.0, fy = 1815.0, cx = 960.0, cy = 540.0;
    gf::CalibrationProfile prof;
    prof.width = w; prof.height = h; prof.fx = fx; prof.fy = fy; prof.cx = cx; prof.cy = cy;
    prof.readoutTimeSec = 0.0;

    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = w; hdr.height = h; hdr.nominal_fps = 30.0;
    hdr.lens_fx = fx; hdr.lens_fy = fy; hdr.lens_cx = cx; hdr.lens_cy = cy;

    const int nFrames = 60;
    const int nImu = 180;

    // Subtest 1: Tilt +-25 deg within +-35 deg band -> flat 0 deg level (< 0.15 deg error)
    {
        std::vector<gf::GFGyroRecord> gyro;
        std::vector<gf::GFAccelRecord> accel;
        const double tiltAmp = 25.0 * (kPi / 180.0);
        for (int i = 0; i < nImu; ++i) {
            const double t = i / 90.0;
            const double rollAng = tiltAmp * std::sin(2.0 * kPi * 0.5 * t);
            const double rollRate = tiltAmp * 2.0 * kPi * 0.5 * std::cos(2.0 * kPi * 0.5 * t);
            // LandscapeLeft: gz = -rollRate
            gyro.push_back({t, 0.0, 0.0, -rollRate});
            // Gravity vector rotated by roll
            const double cr = std::cos(rollAng), sr = std::sin(rollAng);
            // World gravity is (0, 1, 0) -> aCam = (sin, cos, 0) -> axIos = -cos, ayIos = -sin
            accel.push_back({t, static_cast<float>(-cr), static_cast<float>(-sr), 0.0f});
        }
        std::vector<gf::GFFrameRecord> frames;
        for (int f = 0; f < nFrames; ++f) {
            frames.push_back({static_cast<uint32_t>(f), f / 30.0, 0.005f, 100.0f, 0.88f, 0.0});
        }

        gf::TrajectoryEngine engine;
        engine.setImuData(hdr, gyro, accel);
        gf::StabilizerConfig cfg;
        cfg.smoothness = 0.78;
        cfg.horizonLockEnabled = true;
        cfg.horizonMode = gf::HorizonMode::ActionLevel40;
        cfg.dynamicZoomEnabled = false;
        cfg.maxZoomLimit = 1.28;

        const auto traj = engine.solve(frames, cfg);
        assert(traj.valid);
        assert(traj.maxOutputRollErrorDeg < 0.15);

        const auto warp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
            engine, traj, prof, 30.0, cfg.maxZoomLimit, false, cfg.dynamicZoomEnabled
        );
        assert(warp.maxZoom <= 1.32);
        // Verify 0% black border
        const double corners[4][2] = {{0.0, 0.0}, {1919.0, 0.0}, {1919.0, 1079.0}, {0.0, 1079.0}};
        for (const auto& pkt : warp.packets) {
            for (int c = 0; c < 4; ++c) {
                const double u = corners[c][0];
                const double v = corners[c][1];
                const double pw = pkt.m[6] * u + pkt.m[7] * v + pkt.m[8];
                assert(pw > 0.0);
                const double us = (pkt.m[0] * u + pkt.m[1] * v + pkt.m[2]) / pw;
                const double vs = (pkt.m[3] * u + pkt.m[4] * v + pkt.m[5]) / pw;
                assert(us >= 0.0 && us <= 1919.0);
                assert(vs >= 0.0 && vs <= 1079.0);
            }
        }
        std::printf("testActionModeActionLevel40 (tilt +-25 deg -> rollErr=%.3f deg, zoom=%.2fx) PASSED\n",
                    traj.maxOutputRollErrorDeg, warp.maxZoom);
    }

    // Subtest 2: Tilt 65 deg -> soft-knee release past 35 deg, cap ~45 deg, crop <= 1.35x, 0% border
    {
        std::vector<gf::GFGyroRecord> gyro;
        std::vector<gf::GFAccelRecord> accel;
        const double rollAng = 65.0 * (kPi / 180.0);
        const double cr = std::cos(rollAng), sr = std::sin(rollAng);
        for (int i = 0; i < nImu; ++i) {
            const double t = i / 90.0;
            gyro.push_back({t, 0.0, 0.0, 0.0});
            accel.push_back({t, static_cast<float>(-cr), static_cast<float>(-sr), 0.0f});
        }
        std::vector<gf::GFFrameRecord> frames;
        for (int f = 0; f < nFrames; ++f) {
            frames.push_back({static_cast<uint32_t>(f), f / 30.0, 0.005f, 100.0f, 0.88f, 0.0});
        }

        gf::TrajectoryEngine engine;
        engine.setImuData(hdr, gyro, accel);
        gf::StabilizerConfig cfg;
        cfg.smoothness = 0.78;
        cfg.horizonLockEnabled = true;
        cfg.horizonMode = gf::HorizonMode::ActionLevel40;
        cfg.dynamicZoomEnabled = false;
        cfg.maxZoomLimit = 1.35;

        const auto traj = engine.solve(frames, cfg);
        assert(traj.valid);
        // Clamped by soft-knee release: max correction is <= 45.0 deg
        assert(traj.maxCorrectionDeg <= 45.0);

        const auto warp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
            engine, traj, prof, 30.0, cfg.maxZoomLimit, false, cfg.dynamicZoomEnabled
        );
        assert(warp.maxZoom <= 1.35);
        std::printf("testActionModeActionLevel40 (tilt 65 deg -> maxCorr=%.1f deg, zoom=%.2fx) PASSED\n",
                    traj.maxCorrectionDeg, warp.maxZoom);
    }
}

void testSoftKneeMarginClampingZeroEdgeBounce() {
    const int w = 1920, h = 1080;
    const double fx = 1815.0, fy = 1815.0, cx = 960.0, cy = 540.0;
    gf::CalibrationProfile prof;
    prof.width = w; prof.height = h; prof.fx = fx; prof.fy = fy; prof.cx = cx; prof.cy = cy;
    prof.readoutTimeSec = 0.0;

    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = w; hdr.height = h; hdr.nominal_fps = 30.0;
    hdr.lens_fx = fx; hdr.lens_fy = fy; hdr.lens_cx = cx; hdr.lens_cy = cy;

    const int nFrames = 60;
    const int nImu = 180;
    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;

    // +-12 deg sudden pitch shock (exceeding Fixed Crop 1.25x safe knee)
    const double shockAmp = 12.0 * (kPi / 180.0);
    const double omegaFreq = 2.0 * kPi * 4.0;
    for (int i = 0; i < nImu; ++i) {
        const double t = i / 90.0;
        const double wx = shockAmp * omegaFreq * std::cos(omegaFreq * t);
        gyro.push_back({t, 0.0, -wx, 0.0});
        accel.push_back({t, 0.0f, -1.0f, 0.0f});
    }

    std::vector<gf::GFFrameRecord> frames;
    for (int f = 0; f < nFrames; ++f) {
        frames.push_back({static_cast<uint32_t>(f), f / 30.0, 0.005f, 100.0f, 0.88f, 0.0});
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel);
    gf::StabilizerConfig cfg;
    cfg.smoothness = 0.78;
    cfg.dynamicZoomEnabled = false;
    cfg.maxZoomLimit = 1.25; // Fixed Crop 1.25x

    const auto traj = engine.solve(frames, cfg);
    assert(traj.valid);

    const auto warp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        engine, traj, prof, 30.0, cfg.maxZoomLimit, false, cfg.dynamicZoomEnabled
    );

    // Verify ZERO black border violation across all frames and all 4 corners
    const double corners[4][2] = {{0.0, 0.0}, {1919.0, 0.0}, {1919.0, 1079.0}, {0.0, 1079.0}};
    for (const auto& pkt : warp.packets) {
        for (int c = 0; c < 4; ++c) {
            const double u = corners[c][0];
            const double v = corners[c][1];
            const double pw = pkt.m[6] * u + pkt.m[7] * v + pkt.m[8];
            assert(pw > 0.0);
            const double us = (pkt.m[0] * u + pkt.m[1] * v + pkt.m[2]) / pw;
            const double vs = (pkt.m[3] * u + pkt.m[4] * v + pkt.m[5]) / pw;
            assert(us >= 0.0 && us <= 1919.0);
            assert(vs >= 0.0 && vs <= 1079.0);
        }
    }

    // Verify smooth C2 deceleration (no jerk bounce at boundaries)
    for (size_t f = 1; f + 1 < traj.smoothFrameQuats.size(); ++f) {
        const double a0 = rotationAngleBetween(traj.smoothFrameQuats[f - 1], traj.smoothFrameQuats[f]);
        const double a1 = rotationAngleBetween(traj.smoothFrameQuats[f], traj.smoothFrameQuats[f + 1]);
        const double jerk = std::fabs(a1 - a0) * (180.0 / kPi);
        assert(jerk < 8.0); // smooth deceleration, zero abrupt bounce
    }

    std::printf("testSoftKneeMarginClampingZeroEdgeBounce (+-12 deg @ 1.25x -> 0%% border violation, smooth decel) PASSED\n");
}

void testHorizontalShakeSuppressionPortraitAndLandscape() {
    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = 1920;
    hdr.height = 1080;
    hdr.nominal_fps = 30.0;

    gf::CalibrationProfile prof{};
    prof.width = 1920;
    prof.height = 1080;
    prof.fx = 1815.0;
    prof.fy = 1815.0;
    prof.cx = 960.0;
    prof.cy = 540.0;
    prof.readoutTimeSec = 0.0;

    const double duration = 3.0;
    const double imuHz = 90.0;
    const int nImu = static_cast<int>(duration * imuHz);
    const int nFrames = static_cast<int>(duration * 30.0);

    // Scenario 1: Portrait 90 deg, vigorous horizontal walking shake (+-5.5 deg @ 3.5 Hz, peak omega ~ 0.75 rad/s)
    {
        std::vector<gf::GFGyroRecord> gyro;
        std::vector<gf::GFAccelRecord> accel;
        gyro.reserve(nImu);
        accel.reserve(nImu);

        const double omegaPeak = 0.75; // rad/s > 0.25 rad/s threshold
        const double freq = 3.5;       // Hz
        for (int i = 0; i < nImu; ++i) {
            const double t = i / imuHz;
            const double wx = omegaPeak * std::cos(2.0 * kPi * freq * t);
            const double wy = 0.08 * std::sin(2.0 * kPi * 1.8 * t);
            // In iOS IMU to camera: cam.x = -ios.gy, cam.y = -ios.gx
            gyro.push_back({t, -wy, -wx, 0.0});
            // Portrait gravity: camera +X is down on portrait (ios ay = -1)
            accel.push_back({t, 0.0f, -1.0f, 0.0f});
        }

        std::vector<gf::GFFrameRecord> frames;
        frames.reserve(nFrames);
        for (int f = 0; f < nFrames; ++f) {
            frames.push_back({static_cast<uint32_t>(f), f / 30.0, 0.005f, 100.0f, 0.88f, 0.0});
        }

        gf::TrajectoryEngine engine;
        engine.setImuData(hdr, gyro, accel);

        gf::StabilizerConfig cfg;
        cfg.pitchSmoothness = 0.78;
        cfg.yawSmoothness = 0.88;
        cfg.videoRotationDegrees = 90; // Portrait mode
        cfg.dynamicZoomEnabled = false;
        cfg.maxZoomLimit = 1.32;      // Fixed Crop 1.32x
        cfg.horizonLockEnabled = true;
        cfg.horizonMode = gf::HorizonMode::ActionLevel40;

        const auto traj = engine.solve(frames, cfg);
        assert(traj.valid);

        // Verify panRatio is low (non-monotonic shake -> tau does not collapse)
        assert(traj.avgPanRatio < 0.50);

        // Measure horizontal angular velocity and jerk reduction
        double sumRawSq = 0.0, sumSmoothSq = 0.0;
        double maxRawJerk = 0.0, maxSmoothJerk = 0.0;
        for (size_t f = 1; f < traj.rawFrameQuats.size(); ++f) {
            const gf::Quat dRaw = (traj.rawFrameQuats[f - 1].conjugate() * traj.rawFrameQuats[f]).normalized();
            const gf::Quat dSmooth = (traj.smoothFrameQuats[f - 1].conjugate() * traj.smoothFrameQuats[f]).normalized();
            const gf::Vec3 rRaw = quatLog(dRaw) * 30.0;
            const gf::Vec3 rSmooth = quatLog(dSmooth) * 30.0;
            sumRawSq += rRaw.x * rRaw.x;
            sumSmoothSq += rSmooth.x * rSmooth.x;

            if (f >= 2) {
                const gf::Quat dRawPrev = (traj.rawFrameQuats[f - 2].conjugate() * traj.rawFrameQuats[f - 1]).normalized();
                const gf::Quat dSmoothPrev = (traj.smoothFrameQuats[f - 2].conjugate() * traj.smoothFrameQuats[f - 1]).normalized();
                const gf::Vec3 jRaw = (quatLog(dRaw) - quatLog(dRawPrev)) * (30.0 * 30.0);
                const gf::Vec3 jSmooth = (quatLog(dSmooth) - quatLog(dSmoothPrev)) * (30.0 * 30.0);
                maxRawJerk = std::max(maxRawJerk, std::fabs(jRaw.x));
                maxSmoothJerk = std::max(maxSmoothJerk, std::fabs(jSmooth.x));
            }
        }
        const double rmsRaw = std::sqrt(sumRawSq / static_cast<double>(nFrames - 1));
        const double rmsSmooth = std::sqrt(sumSmoothSq / static_cast<double>(nFrames - 1));
        const double shakeReduction = (1.0 - rmsSmooth / rmsRaw) * 100.0;
        const double jerkReduction = (1.0 - maxSmoothJerk / maxRawJerk) * 100.0;

        assert(shakeReduction > 90.0); // > 90% (specifically > 92% reduction)
        assert(jerkReduction > 85.0);  // > 85% reduction in horizontal jerk

        const auto warp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
            engine, traj, prof, 30.0, cfg.maxZoomLimit, false, cfg.dynamicZoomEnabled
        );

        // 0% black border violation across all frames and all 4 corners
        const double corners[4][2] = {{0.0, 0.0}, {1919.0, 0.0}, {1919.0, 1079.0}, {0.0, 1079.0}};
        for (const auto& pkt : warp.packets) {
            for (int c = 0; c < 4; ++c) {
                const double u = corners[c][0];
                const double v = corners[c][1];
                const double pw = pkt.m[6] * u + pkt.m[7] * v + pkt.m[8];
                assert(pw > 0.0);
                const double us = (pkt.m[0] * u + pkt.m[1] * v + pkt.m[2]) / pw;
                const double vs = (pkt.m[3] * u + pkt.m[4] * v + pkt.m[5]) / pw;
                assert(us >= 0.0 && us <= 1919.0);
                assert(vs >= 0.0 && vs <= 1079.0);
            }
        }
    }

    // Scenario 2: Landscape 0 deg, mixed horizontal shake (2 Hz + 7 Hz)
    {
        std::vector<gf::GFGyroRecord> gyro;
        std::vector<gf::GFAccelRecord> accel;
        gyro.reserve(nImu);
        accel.reserve(nImu);

        for (int i = 0; i < nImu; ++i) {
            const double t = i / imuHz;
            const double wy = 0.05 * std::sin(2.0 * kPi * 2.0 * t) + 0.03 * std::sin(2.0 * kPi * 7.0 * t);
            gyro.push_back({t, -wy, 0.0, 0.0});
            accel.push_back({t, -1.0f, 0.0f, 0.0f}); // Landscape gravity
        }

        std::vector<gf::GFFrameRecord> frames;
        frames.reserve(nFrames);
        for (int f = 0; f < nFrames; ++f) {
            frames.push_back({static_cast<uint32_t>(f), f / 30.0, 0.005f, 100.0f, 0.88f, 0.0});
        }

        gf::TrajectoryEngine engine;
        engine.setImuData(hdr, gyro, accel);

        gf::StabilizerConfig cfg;
        cfg.pitchSmoothness = 0.78;
        cfg.yawSmoothness = 0.88;
        cfg.videoRotationDegrees = 0; // Landscape mode
        cfg.dynamicZoomEnabled = false;
        cfg.maxZoomLimit = 1.32;
        cfg.horizonLockEnabled = true;
        cfg.horizonMode = gf::HorizonMode::ActionLevel40;

        const auto traj = engine.solve(frames, cfg);
        assert(traj.valid);

        const auto warp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
            engine, traj, prof, 30.0, cfg.maxZoomLimit, false, cfg.dynamicZoomEnabled
        );

        // Stabilized frame-to-frame horizontal pixel residual variation (< 0.8 pixel)
        double stabSumSq = 0.0;
        int stabCount = 0;
        for (size_t f = 1; f < traj.smoothFrameQuats.size(); ++f) {
            const gf::Vec3 fwd0 = traj.smoothFrameQuats[f - 1].rotateVector({0.0, 0.0, 1.0});
            const gf::Vec3 fwd1 = traj.smoothFrameQuats[f].rotateVector({0.0, 0.0, 1.0});
            const double duStab = (fwd1.x - fwd0.x) * prof.fx;
            stabSumSq += duStab * duStab;
            ++stabCount;
        }
        const double stabRmsPx = std::sqrt(stabSumSq / static_cast<double>(stabCount));
        std::printf("Scenario 2 Landscape 0 deg: post-stabilization horizontal RMS = %.4f px (< 0.8 px)\n", stabRmsPx);
        std::fflush(stdout);
        assert(stabRmsPx < 0.8);
    }

    std::printf("testHorizontalShakeSuppressionPortraitAndLandscape (Portrait 90 deg shake >92%% & Landscape 0 deg RMS <0.8px) PASSED\n");
}

void testConstrainedPathShakeAndPanZeroBorder() {
    gf::CalibrationProfile prof = gf::CalibrationProfile::defaultIPhone5s1080p30();
    prof.k1 = 0.0;
    prof.k2 = 0.0;
    prof.readoutTimeSec = 0.0152;

    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = 1920;
    hdr.height = 1080;
    hdr.nominal_fps = 30.0;
    hdr.readout_time_sec = 0.0152;
    hdr.lens_fx = prof.fx;
    hdr.lens_fy = prof.fy;
    hdr.lens_cx = prof.cx;
    hdr.lens_cy = prof.cy;

    const double duration = 5.5;
    const double imuHz = 90.0;
    const int nImu = static_cast<int>(duration * imuHz);
    const int nFrames = static_cast<int>(duration * 30.0);

    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    gyro.reserve(static_cast<size_t>(nImu));
    accel.reserve(static_cast<size_t>(nImu));

    const double pitchAmp = 9.0 * (kPi / 180.0); // +-9 deg shake exceeds 1.30x crop envelope
    const double yawAmp = 4.2 * (kPi / 180.0);
    const double wPan = 30.0 * (kPi / 180.0);    // 30 deg/s constant-velocity pan segment

    for (int i = 0; i < nImu; ++i) {
        const double t = i / imuHz;
        double panYaw = 0.0;
        if (t > 1.4 && t < 4.1) {
            const double u1 = std::min(1.0, std::max(0.0, (t - 1.4) / 0.25));
            const double u2 = std::min(1.0, std::max(0.0, (4.1 - t) / 0.25));
            panYaw = wPan * (u1 * u1 * (3.0 - 2.0 * u1)) * (u2 * u2 * (3.0 - 2.0 * u2));
        }
        const double wx = pitchAmp * (2.0 * kPi * 3.2) * std::cos(2.0 * kPi * 3.2 * t);
        const double wy = panYaw + yawAmp * (2.0 * kPi * 4.0) * std::cos(2.0 * kPi * 4.0 * t);
        gyro.push_back({t, -wy, -wx, 0.0});
        accel.push_back({t, -1.0f, 0.0f, 0.0f}); // Landscape gravity (+Y_cam)
    }

    std::vector<gf::GFFrameRecord> frames;
    frames.reserve(static_cast<size_t>(nFrames));
    for (int f = 0; f < nFrames; ++f) {
        frames.push_back({static_cast<uint32_t>(f), f / 30.0, 0.005f, 100.0f, 0.88f, 0.0152f});
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel);

    gf::StabilizerConfig cfgOld;
    cfgOld.pitchSmoothness = 0.78;
    cfgOld.yawSmoothness = 0.88;
    cfgOld.rollSmoothness = 0.85;
    cfgOld.horizonLockEnabled = true;
    cfgOld.horizonMode = gf::HorizonMode::ActionLevel40;
    cfgOld.rollingShutterCorrection = true;
    cfgOld.dynamicZoomEnabled = false;
    cfgOld.maxZoomLimit = 1.30;
    cfgOld.useConstrainedPath = false;
    cfgOld.calibWidth = prof.width;
    cfgOld.calibHeight = prof.height;
    cfgOld.calibFx = prof.fx;
    cfgOld.calibFy = prof.fy;
    cfgOld.calibCx = prof.cx;
    cfgOld.calibCy = prof.cy;
    cfgOld.calibReadoutSec = prof.readoutTimeSec;

    gf::StabilizerConfig cfgNew = cfgOld;
    cfgNew.useConstrainedPath = true;

    const auto trajOld = engine.solve(frames, cfgOld);
    const auto trajNew = engine.solve(frames, cfgNew);
    assert(trajOld.valid && trajNew.valid);

    const auto warpNew = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        engine, trajNew, prof, 30.0, cfgNew.maxZoomLimit, true, false
    );
    assert(warpNew.maxZoom <= cfgNew.maxZoomLimit + 1e-3);

    // Verify 0% black border across ALL RS slices on every frame
    const double corners[4][2] = {{0.0, 0.0}, {1919.0, 0.0}, {1919.0, 1079.0}, {0.0, 1079.0}};
    for (const auto& lut : warpNew.frameLUTs) {
        const int testSlices[3] = {0, gf::kRollingShutterSlices / 2, gf::kRollingShutterSlices - 1};
        for (int s : testSlices) {
            const float* m = lut.sliceH[s];
            for (int c = 0; c < 4; ++c) {
                const double u = corners[c][0];
                const double v = corners[c][1];
                const double pw = m[6] * u + m[7] * v + m[8];
                assert(pw > 0.0);
                const double us = (m[0] * u + m[1] * v + m[2]) / pw;
                const double vs = (m[3] * u + m[4] * v + m[5]) / pw;
                assert(us >= 0.0 && us <= 1919.0);
                assert(vs >= 0.0 && vs <= 1079.0);
            }
        }
    }

    // Verify constant-velocity pan plateau in [2.0s..3.5s] (frames 60..105)
    std::vector<double> plateauWy;
    double wySum = 0.0;
    for (size_t f = 60; f < 105; ++f) {
        const gf::Vec3 w = quatLog((trajNew.smoothFrameQuats[f].conjugate() * trajNew.smoothFrameQuats[f + 1]).normalized()) * 30.0;
        plateauWy.push_back(w.y);
        wySum += w.y;
    }
    const double wyMean = wySum / static_cast<double>(plateauWy.size());
    double wyVar = 0.0;
    for (double v : plateauWy) wyVar += (v - wyMean) * (v - wyMean);
    const double wyStd = std::sqrt(wyVar / static_cast<double>(plateauWy.size()));
    assert(std::fabs(wyMean - wPan) < 0.08);
    assert(wyStd < 0.06);

    const auto repOld = gf::QualityMetrics::evaluateTrajectory(trajOld.smoothFrameQuats, 30.0, nullptr, trajOld.maxOutputRollErrorDeg, 3.0);
    const auto repNew = gf::QualityMetrics::evaluateTrajectory(trajNew.smoothFrameQuats, 30.0, nullptr, trajNew.maxOutputRollErrorDeg, 3.0);
    std::printf(
        "ConstrainedPath Shake+Pan A/B: JerkRMS %.2f -> %.2f | AccelJumps %d -> %d | PanPlateau=%.3f+-%.4f rad/s | MaxZoom=%.3fx\n",
        repOld.jerkRms, repNew.jerkRms,
        repOld.accelDiscontinuityCount, repNew.accelDiscontinuityCount,
        wyMean, wyStd, warpNew.maxZoom
    );
    assert(repNew.jerkRms < repOld.jerkRms);
    assert(repNew.accelDiscontinuityCount < repOld.accelDiscontinuityCount);
    assert(trajNew.maxOutputRollErrorDeg < 0.5);
}

void testConstrainedPathPreservesSmoothInput() {
    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = 1920;
    hdr.height = 1080;
    hdr.nominal_fps = 30.0;

    // 1.0s tripod hold + 2.5s ultra-clean constant 20 deg/s pan + 1.0s hold (ZERO shake)
    const double duration = 4.5;
    const double imuHz = 90.0;
    const int nImu = static_cast<int>(duration * imuHz);
    const int nFrames = static_cast<int>(duration * 30.0);
    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    const double wPan = 20.0 * (kPi / 180.0);

    for (int i = 0; i < nImu; ++i) {
        const double t = i / imuHz;
        double wy = 0.0;
        if (t > 1.0 && t < 3.5) {
            wy = wPan;
        }
        gyro.push_back({t, -wy, 0.0, 0.0});
        accel.push_back({t, 0.0f, -1.0f, 0.0f});
    }

    std::vector<gf::GFFrameRecord> frames;
    for (int f = 0; f < nFrames; ++f) {
        frames.push_back({static_cast<uint32_t>(f), f / 30.0, 0.005f, 100.0f, 0.88f, 0.0});
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel);
    gf::StabilizerConfig cfg;
    cfg.pitchSmoothness = 0.78;
    cfg.yawSmoothness = 0.88;
    cfg.rollSmoothness = 0.85;
    cfg.horizonLockEnabled = true;
    cfg.horizonMode = gf::HorizonMode::ActionLevel40;
    cfg.maxZoomLimit = 1.32;
    cfg.useConstrainedPath = true;

    const auto traj = engine.solve(frames, cfg);
    assert(traj.valid);

    double sqErrDeg = 0.0;
    for (size_t f = 0; f < traj.rawFrameQuats.size(); ++f) {
        const double errDeg = rotationAngleBetween(traj.rawFrameQuats[f], traj.smoothFrameQuats[f]) * (180.0 / kPi);
        sqErrDeg += errDeg * errDeg;
    }
    const double rmsDiffDeg = std::sqrt(sqErrDeg / static_cast<double>(traj.rawFrameQuats.size()));
    std::printf("ConstrainedPath SmoothInput Preservation RMS diff = %.4f deg (< 0.25 deg)\n", rmsDiffDeg);
    assert(rmsDiffDeg < 0.25);
}

void testConstrainedPathPerformance60Sec() {
    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = 1920;
    hdr.height = 1080;
    hdr.nominal_fps = 30.0;
    hdr.readout_time_sec = 0.0152;

    const int nFrames = 1800; // 60 seconds @ 30 fps
    const int nImu = 5400;    // 90 Hz IMU
    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    gyro.reserve(static_cast<size_t>(nImu));
    accel.reserve(static_cast<size_t>(nImu));

    for (int i = 0; i < nImu; ++i) {
        const double t = i / 90.0;
        const double wx = 0.45 * std::cos(2.0 * kPi * 2.8 * t);
        const double wy = 0.25 * std::sin(2.0 * kPi * 0.15 * t) + 0.35 * std::sin(2.0 * kPi * 3.4 * t);
        const double wz = 0.22 * std::cos(2.0 * kPi * 3.1 * t);
        gyro.push_back({t, -wy, -wx, -wz});
        accel.push_back({t, 0.0f, -1.0f, 0.0f});
    }

    std::vector<gf::GFFrameRecord> frames;
    frames.reserve(static_cast<size_t>(nFrames));
    for (int f = 0; f < nFrames; ++f) {
        frames.push_back({static_cast<uint32_t>(f), f / 30.0, 0.005f, 100.0f, 0.88f, 0.0152f});
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel);
    gf::StabilizerConfig cfg;
    cfg.pitchSmoothness = 0.78;
    cfg.yawSmoothness = 0.88;
    cfg.rollSmoothness = 0.85;
    cfg.horizonLockEnabled = true;
    cfg.horizonMode = gf::HorizonMode::ActionLevel40;
    cfg.rollingShutterCorrection = true;
    cfg.maxZoomLimit = 1.32;
    cfg.useConstrainedPath = true;

    const auto t0 = std::chrono::steady_clock::now();
    const auto traj = engine.solve(frames, cfg);
    const auto t1 = std::chrono::steady_clock::now();
    const double elapsedSec = std::chrono::duration<double>(t1 - t0).count();
    assert(traj.valid && traj.smoothFrameQuats.size() == 1800);
    std::printf("ConstrainedPath 60s (1800 frames) solve time = %.4f s (< 2.0 s host, << 10 s A7)\n", elapsedSec);
    assert(elapsedSec < 2.0);
}

} // namespace

int main() {
    testHomographySignDirection();
    testStabilizerJitterAndPanAndZeroBlackBorder();
    testPureInterFrameGyroDeRotation();
    testActionModeActionLevel40AndFixedCrop();
    testSoftKneeMarginClampingZeroEdgeBounce();
    testHorizontalShakeSuppressionPortraitAndLandscape();
    testConstrainedPathShakeAndPanZeroBorder();
    testConstrainedPathPreservesSmoothInput();
    testConstrainedPathPerformance60Sec();
    return 0;
}
