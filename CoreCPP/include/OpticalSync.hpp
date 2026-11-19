#pragma once

#include "CalibrationProfile.hpp"
#include "SidecarFormat.hpp"
#include "TrajectoryEngine.hpp"

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
    double signedReadoutTimeMs = 15.2;
    bool hasReadoutEstimate = false;
    double effectiveFocalScale = 1.0;
    bool hasFocalScaleEstimate = false;
    double rollGyroScale = 1.0;
    bool hasRollScaleEstimate = false;
    std::string diagnosticMessage;
};

struct AxisPreset {
    const char* name;
    double m[9]; // Row-major 3x3 orthogonal rotation (det = +1)
};

struct HybridResidualResult {
    bool valid = false;
    std::vector<double> shiftX; // Virtual principal-point X offset (px)
    std::vector<double> shiftY; // Virtual principal-point Y offset (px)
    double maxAmplitudePx = 0.0;
    double rmsResidualPx = 0.0;
};

class HybridResidualTracker {
public:
    void reset(const CalibrationProfile& profile, const TrajectoryResult& traj, bool motoActionMode = false);
    void pushFrame(const OpticalFlowFrame& frame);
    HybridResidualResult finalize(double nominalFps) const;

private:
    CalibrationProfile profile_{};
    std::vector<Quat> rawQuats_;
    std::vector<Quat> smoothQuats_;
    OpticalFlowFrame prevFrame_{};
    Quat prevQCorr_ = Quat::identity();
    bool hasPrev_ = false;
    bool motoActionMode_ = false;
    size_t frameIdx_ = 0;
    std::vector<double> deltaU_;
    std::vector<double> deltaV_;
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
        double& outWz,
        bool motoActionMode = false
    );

    /// Sweeps axis presets and time offsets in [-80ms, +80ms] to find the highest NCC
    /// between visual angular velocity and IMU gyro measurements.
    static OpticalSyncResult solveSyncAndAxes(
        const std::vector<OpticalFlowFrame>& frames,
        const GFFileHeader& header,
        const std::vector<GFGyroRecord>& gyroRecords,
        bool motoActionMode = false
    );

    /// Computes 2D translational residual shifts after 3D gyro rotation compensation.
    static HybridResidualResult estimateHybridResidualShifts(
        const std::vector<OpticalFlowFrame>& frames,
        const CalibrationProfile& profile,
        const TrajectoryResult& traj,
        double nominalFps
    );
};

} // namespace gf
