#pragma once

#include "CalibrationProfile.hpp"
#include "Quaternion.hpp"
#include "SidecarFormat.hpp"

#include <vector>

namespace gf {

struct StabilizerConfig {
    double smoothness = 0.65;        // [0.0, 1.0] -> maps to tau_base in [0.05s, 1.80s]
    double velocityDamping = 1.0;    // Reduces tau during intentional fast pans
    double timeOffsetMs = 0.0;       // Fine sync offset in [-50.0, +50.0] ms
    double maxZoomLimit = 1.35;      // <= 1.0 disables cap (unlimited safe zoom)
    int cameraOrientationMode = 0;   // 0 = LandscapeLeft (standard iPhone 5s rear cam)
};

struct TrajectoryResult {
    std::vector<Quat> rawFrameQuats;
    std::vector<Quat> smoothFrameQuats;
    std::vector<double> frameTimestampsSec;
    double stabilityImprovementPct = 0.0;
    double rawAngularAccelRms = 0.0;
    double smoothAngularAccelRms = 0.0;
    bool valid = false;
};

class TrajectoryEngine {
public:
    void setImuData(
        const GFFileHeader& header,
        const std::vector<GFGyroRecord>& gyroRecords,
        const std::vector<GFAccelRecord>& accelRecords
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
    double nominalFps_ = 30.0;
};

} // namespace gf
