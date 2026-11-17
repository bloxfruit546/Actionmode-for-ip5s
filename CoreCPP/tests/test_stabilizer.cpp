#include "FrameWarpCalculator.hpp"
#include "TrajectoryEngine.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

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

    gf::StabilizerConfig cfg;
    cfg.smoothness = 0.50;
    cfg.velocityDamping = 1.0;
    cfg.maxZoomLimit = 0.0; // Unlimited safe zoom -> 0% black border guaranteed

    const gf::TrajectoryResult traj = engine.solve(frames, cfg);
    assert(traj.valid);
    assert(traj.rawFrameQuats.size() == frames.size());
    assert(traj.smoothFrameQuats.size() == frames.size());
    assert(traj.stabilityImprovementPct > 70.0);

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

} // namespace

int main() {
    testHomographySignDirection();
    testStabilizerJitterAndPanAndZeroBlackBorder();
    return 0;
}
