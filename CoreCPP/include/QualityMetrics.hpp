#pragma once

#include "Quaternion.hpp"
#include "VirtualGyroEngine.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace gf {

struct PtsCheckResult {
    size_t inputCount = 0;
    size_t outputCount = 0;
    size_t nonMonotonicCount = 0;
    size_t duplicateFrameCount = 0;
    size_t writerDroppedCount = 0;
    double minDtMs = 0.0;
    double maxDtMs = 0.0;
    double meanDtMs = 0.0;
    double stdDevDtMs = 0.0;
    double maxInputOutputStepDriftMs = 0.0;
    bool passed = false;
};

struct QualityReport {
    double highFreqOmegaEnergyAbove2Hz = 0.0; // (rad/s)^2 spectral energy above 2.0 Hz
    double highFreqOmegaRmsAbove2Hz = 0.0;    // rad/s RMS above 2.0 Hz
    double visualOmegaResRms = 0.0;           // rad/s overall visual residual omega RMS
    double jerkRms = 0.0;                     // rad/s^3 RMS of 3rd derivative of camera path
    double liuStabilityScore = 1.0;           // [0, 1] min-axis energy fraction in lowest 6 DFT bins
    double liuStabilityScoreMean = 1.0;       // [0, 1] mean-axis Liu stability score
    double meanCroppingRatio = 1.0;           // mean(1.0 / zoomUsed) in (0, 1]
    double minCroppingRatio = 1.0;            // 1.0 / maxZoom
    double avgZoom = 1.0;
    double maxZoom = 1.0;
    int accelDiscontinuityCount = 0;          // count of frames with |delta a| > threshold
    double maxAccelJumpRadPerSec2 = 0.0;
    double accelJumpThresholdRadPerSec2 = 3.0;
    double maxHorizonErrorDeg = 0.0;
    double meanInlierRatio = 1.0;
    double calibResidualRmsBefore = 0.0;
    double calibResidualRmsAfter = 0.0;
    size_t frameCount = 0;
    double nominalFps = 30.0;
    bool hasVisualMetrics = false;
    PtsCheckResult ptsCheck;
};

class QualityMetrics {
public:
    static std::vector<Vec3> computeRelativeAngularVelocities(
        const std::vector<Quat>& quats,
        double nominalFps
    );

    static QualityReport evaluateOmegaSequence(
        const std::vector<Vec3>& omegaRes,
        double nominalFps,
        const std::vector<double>* zoomUsed = nullptr,
        double maxHorizonErrorDeg = 0.0,
        double accelJumpThresholdRadPerSec2 = 3.0
    );

    static QualityReport evaluateTrajectory(
        const std::vector<Quat>& smoothQuats,
        double nominalFps,
        const std::vector<double>* zoomUsed = nullptr,
        double maxHorizonErrorDeg = 0.0,
        double accelJumpThresholdRadPerSec2 = 3.0
    );

    static PtsCheckResult validatePtsIntegrity(
        const std::vector<double>& inputPtsSec,
        const std::vector<double>& outputPtsSec,
        double nominalFps
    );

    /// Zero-phase 2nd-order Butterworth LPF (< 1 Hz, default 0.65 Hz) for temporally-stable CAS sharpening.
    static std::vector<double> lowPassFilterZoomSub1Hz(
        const std::vector<double>& zoomCurve,
        double fps,
        double cutoffHz = 0.65
    );

    static std::string toJson(const QualityReport& report, const std::string& clipName = "");
    static std::string ptsToJson(const PtsCheckResult& pts);
};

/**
 * Streaming post-render video quality analyzer (keeps only 2 frames in RAM).
 * Reuses VirtualGyroEngine (Pyramidal LK + RANSAC) on rendered output frames to measure omega_res(t).
 */
class VideoQualityAnalyzer {
public:
    explicit VideoQualityAnalyzer(const VirtualGyroConfig& config);

    bool pushRenderedLumaFrame(const uint8_t* luma, int width, int height, double ptsSec);

    QualityReport finalize(
        const std::vector<double>* zoomUsed = nullptr,
        double maxHorizonErrorDeg = 0.0,
        double accelJumpThresholdRadPerSec2 = 3.0
    ) const;

    const std::vector<VirtualGyroEngine::VisualOmegaSample>& visualOmegaSamples() const {
        return virtualGyro_.estimatedSamples();
    }

private:
    VirtualGyroConfig config_;
    VirtualGyroEngine virtualGyro_;
};

} // namespace gf
