#pragma once

#include "CalibrationProfile.hpp"
#include "Quaternion.hpp"
#include "SidecarFormat.hpp"

#include <vector>

namespace gf {

struct StabilizerConfig {
    double smoothness = 0.65;        // Legacy fallback if per-axis not overridden
    double pitchSmoothness = 0.65;   // [0.0, 1.0]
    double yawSmoothness = 0.65;     // [0.0, 1.0]
    double rollSmoothness = 0.75;    // [0.0, 1.0]
    double velocityDamping = 1.0;    // Reduces tau during intentional fast pans
    double timeOffsetMs = 0.0;       // Fine sync offset in [-80.0, +80.0] ms
    double maxZoomLimit = 1.35;      // <= 1.0 disables cap (unlimited safe zoom)
    bool horizonLockEnabled = false;
    double horizonLockAmount = 1.0;  // [0.0, 1.0]
    double horizonTiltOffsetDeg = 0.0;
    bool rollingShutterCorrection = true;
    bool dynamicZoomEnabled = true;
    int videoRotationDegrees = 0;    // 0, 90, 180, 270 from AVAssetTrack.preferredTransform
    double axisMatrix[9] = {0, -1, 0, -1, 0, 0, 0, 0, -1};
    bool hasCustomAxisMatrix = false;
};

struct TrajectoryResult {
    std::vector<Quat> rawFrameQuats;
    std::vector<Quat> smoothFrameQuats;
    std::vector<double> frameTimestampsSec;
    double stabilityImprovementPct = 0.0;
    double rawAngularAccelRms = 0.0;
    double smoothAngularAccelRms = 0.0;
    double maxInputRollDeg = 0.0;
    double maxOutputRollErrorDeg = 0.0;
    double maxCorrectionDeg = 0.0;
    int detectedGravityQuadrantDeg = 0;
    bool valid = false;
};

class TrajectoryEngine {
public:
    void setImuData(
        const GFFileHeader& header,
        const std::vector<GFGyroRecord>& gyroRecords,
        const std::vector<GFAccelRecord>& accelRecords,
        const double* customAxisMatrix = nullptr
    );

    Quat sampleRawQuaternionAt(double timestampSec) const;

    TrajectoryResult solve(
        const std::vector<GFFrameRecord>& frames,
        const StabilizerConfig& config
    ) const;

private:
    struct ImuSample {
        double timestampSec;
        Quat quat;
        Vec3 omegaCam;
    };

    std::vector<ImuSample> integratedSamples_;
    Vec3 initGravityCam_{0.0, 1.0, 0.0};
    double nominalFps_ = 30.0;
};

} // namespace gf
