#pragma once

#include "CalibrationProfile.hpp"
#include "Quaternion.hpp"
#include "SidecarFormat.hpp"

#include <vector>

namespace gf {

struct RefinedCalibration;

enum class HorizonMode : int {
    Off = 0,
    ActionLevel40 = 1,  // ±40° Action Horizon Leveling (soft-knee release past ±35°)
    Full360Lock = 2     // Full 360° gimbal lock
};

struct StabilizerConfig {
    double smoothness = 0.78;        // Action Mode default
    double pitchSmoothness = -1.0;   // < 0 means use smoothness
    double yawSmoothness = -1.0;     // < 0 means use smoothness
    double rollSmoothness = -1.0;    // < 0 means use smoothness
    double velocityDamping = 1.0;    // Reduces tau during intentional fast pans
    double timeOffsetMs = 0.0;       // Fine sync offset in [-80.0, +80.0] ms
    double maxZoomLimit = 0.0;       // <= 1.0 disables cap (unlimited safe zoom). Default 0.0
    bool horizonLockEnabled = false; // Off by default in raw struct; enabled in UI/Presets
    double horizonLockAmount = 1.0;  // [0.0, 1.0]
    double horizonTiltOffsetDeg = 0.0;
    HorizonMode horizonMode = HorizonMode::Full360Lock; // When horizonLockEnabled is true, defaults to Full 360 unless ActionLevel40 is set
    bool rollingShutterCorrection = true;
    bool dynamicZoomEnabled = true;  // Dynamic zoom by default in C++ core; Action Mode preset uses Fixed Crop
    bool motoActionMode = false;
    bool useConstrainedPath = true;  // Task B: L1 ConstrainedPathOptimizer (A/B fallback when false)
    int videoRotationDegrees = 0;    // 0, 90, 180, 270 from AVAssetTrack.preferredTransform
    uint32_t calibWidth = 0;         // 0 = use header/default (1920)
    uint32_t calibHeight = 0;        // 0 = use header/default (1080)
    double calibFx = 0.0;            // <= 0 = use header/default (1815.0)
    double calibFy = 0.0;
    double calibCx = 0.0;
    double calibCy = 0.0;
    double calibReadoutSec = -1.0;   // < 0 = use frame/header readout
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
    double maxSwingCapDeg = 11.0;
    int detectedGravityQuadrantDeg = 0;
    double avgPanRatio = 0.0;
    bool valid = false;
};

class TrajectoryEngine {
public:
    void setImuData(
        const GFFileHeader& header,
        const std::vector<GFGyroRecord>& gyroRecords,
        const std::vector<GFAccelRecord>& accelRecords,
        const double* customAxisMatrix = nullptr,
        bool motoActionMode = false,
        double rollGyroScale = 1.0,
        const RefinedCalibration* refinedCalib = nullptr
    );

    Quat sampleRawQuaternionAt(double timestampSec) const;
    Vec3 sampleGravityWorldAt(double timestampSec) const;

    TrajectoryResult solve(
        const std::vector<GFFrameRecord>& frames,
        const StabilizerConfig& config
    ) const;

private:
    struct ImuSample {
        double timestampSec;
        Quat quat;
        Vec3 omegaCam;
        Vec3 gravityWorld;
    };

    std::vector<ImuSample> integratedSamples_;
    Vec3 initGravityCam_{0.0, 1.0, 0.0};
    double nominalFps_ = 30.0;
    uint32_t hdrWidth_ = 1920;
    uint32_t hdrHeight_ = 1080;
    double hdrFx_ = 1815.0;
    double hdrFy_ = 1815.0;
    double hdrCx_ = 960.0;
    double hdrCy_ = 540.0;
    double hdrReadoutSec_ = 0.0152;
};

} // namespace gf
