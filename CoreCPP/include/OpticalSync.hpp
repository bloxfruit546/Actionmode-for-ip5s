#pragma once

#include "SidecarFormat.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace gf {

struct OpticalFlowFrame {
    double timestampSec = 0.0;
    int width = 0;
    int height = 0;
    std::vector<uint8_t> luma;
};

struct OpticalSyncResult {
    bool valid = false;
    double bestOffsetMs = 0.0;
    double bestNCC = 0.0;
    int bestAxisPresetIndex = 0;
    std::string axisPresetName;
    double axisMatrix[9] = {0, -1, 0, -1, 0, 0, 0, 0, -1};
    std::string diagnosticMessage;
};

struct AxisPreset {
    const char* name;
    double m[9]; // Row-major 3x3 orthogonal rotation (det = +1)
};

class OpticalSync {
public:
    static const std::vector<AxisPreset>& getAxisPresets();

    /// Estimates visual angular velocity (rad/s) in pixel-buffer camera coordinates
    /// between consecutive frames f0 and f1 using 6x4 grid block matching + RANSAC/IRLS.
    static bool estimateVisualOmega(
        const OpticalFlowFrame& f0,
        const OpticalFlowFrame& f1,
        double fx,
        double fy,
        double cx,
        double cy,
        double& outWx,
        double& outWy,
        double& outWz
    );

    /// Sweeps axis presets and time offsets in [-80ms, +80ms] to find the highest NCC
    /// between visual angular velocity and IMU gyro measurements.
    static OpticalSyncResult solveSyncAndAxes(
        const std::vector<OpticalFlowFrame>& frames,
        const GFFileHeader& header,
        const std::vector<GFGyroRecord>& gyroRecords
    );
};

} // namespace gf
