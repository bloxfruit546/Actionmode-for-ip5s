#include "FrameWarpCalculator.hpp"
#include "TrajectoryEngine.hpp"
#include "TranslationalStabilizer2D.hpp"

#include <cassert>
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

} // namespace

int main() {
    testHomographySignDirection();
    testStabilizerJitterAndPanAndZeroBlackBorder();
    testPureInterFrameGyroDeRotation();
    testActionModeActionLevel40AndFixedCrop();
    testSoftKneeMarginClampingZeroEdgeBounce();
    return 0;
}
