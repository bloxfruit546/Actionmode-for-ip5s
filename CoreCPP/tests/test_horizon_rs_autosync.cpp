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
    // Top row slice (0) and bottom row slice (15) must differ due to 15.2ms readout
    const float diff = std::fabs(warp.frameLUTs[10].sliceH[0][2] - warp.frameLUTs[10].sliceH[15][2]);
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

} // namespace

int main() {
    testHorizonLock360();
    testRollingShutterSlices();
    testOpticalSyncOffsetAndAxisDetection();
    return 0;
}
