#pragma once

#include "CalibrationProfile.hpp"
#include "OpticalSync.hpp"
#include "SidecarFormat.hpp"
#include "VirtualGyroEngine.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace gf {

struct CalibrationModePrior {
    bool hasPrior = false;
    double readoutMs = 15.2;
    double focalScale = 1.0;
    double scale[3] = {1.0, 1.0, 1.0};
};

struct RefinedCalibration {
    double offsetA = 0.0;               // Time offset at referenceStartSec (ms)
    double offsetB = 0.0;               // Linear clock drift (ms/s): delta_ms(t) = offsetA + offsetB * (t - referenceStartSec)
    double scale[3] = {1.0, 1.0, 1.0};  // Per-axis gyro scale in camera frame (wx_gyro = scale[i] * wx_vis + bias[i])
    double bias[3] = {0.0, 0.0, 0.0};   // Per-axis total gyro bias in camera frame (rad/s)
    double readoutMs = 15.2;            // Signed rolling-shutter readout time (ms)
    double focalScale = 1.0;            // Effective focal length scale factor
    double confidence = 0.0;            // Overall confidence [0.0, 1.0]
    double residualRmsBefore = 0.0;     // RMS angular velocity residual before refine (rad/s)
    double residualRmsAfter = 0.0;      // RMS angular velocity residual after refine (rad/s)
    double referenceStartSec = 0.0;     // Reference frame timestamp t_ref (s)
    int axisPresetIndex = 0;
    double axisMatrix[9] = {0, -1, 0, -1, 0, 0, 0, 0, -1};
    bool usedPriorFallback = false;
    bool valid = false;

    std::string toJson() const;
    static bool fromJson(const std::string& jsonText, RefinedCalibration& outCal);
    static bool saveJsonFile(const std::string& path, const RefinedCalibration& cal);
    static bool loadJsonFile(const std::string& path, RefinedCalibration& outCal);
};

class CalibrationRefiner {
public:
    CalibrationRefiner();

    void reset(
        const GFFileHeader& header,
        const std::vector<GFGyroRecord>& gyroRecords,
        const std::vector<GFFrameRecord>& frames,
        const CalibrationModePrior* prior = nullptr,
        const OpticalSyncResult* initialSync = nullptr,
        bool motoActionMode = false
    );

    /// Streaming frame intake (keeps only 2 downsampled 4x pyramids in RAM).
    bool pushDownsampledLumaFrame(
        const uint8_t* luma,
        int width,
        int height,
        double ptsSec,
        size_t frameIndex
    );

    RefinedCalibration finalize();

    static double computeMedian(std::vector<double> values);
    static CalibrationModePrior computeModePriorFromHistory(
        const std::vector<double>& readoutMsHist,
        const std::vector<double>& focalScaleHist,
        const std::vector<double>& scaleXHist,
        const std::vector<double>& scaleYHist,
        const std::vector<double>& scaleZHist,
        double defaultReadoutMs = 15.2
    );

private:
    struct VisPairSample {
        double t0 = 0.0;
        double t1 = 0.0;
        double tMid = 0.0;
        double dt = 0.0;
        double wxVis = 0.0;
        double wyVis = 0.0;
        double wzVis = 0.0;
        double inlierRatio = 0.0;
        int inlierCount = 0;
        double meanRow0 = 0.0;
        double meanRow1 = 0.0;
        bool valid = false;
    };

    struct PointPairFrame {
        double t0 = 0.0;
        double t1 = 0.0;
        double motionScore = 0.0;
        std::vector<VirtualGyroEngine::TrackedPointPair> pts;
    };

    GFFileHeader header_{};
    std::vector<GFGyroRecord> sortedGyro_;
    std::vector<GFFrameRecord> frames_;
    CalibrationModePrior prior_{};
    OpticalSyncResult initialSync_{};
    bool hasInitialSync_ = false;
    bool motoActionMode_ = false;

    int firstFrameW_ = 0;
    int firstFrameH_ = 0;
    bool hasPrev_ = false;
    double prevCenterSec_ = 0.0;
    VirtualGyroEngine::FramePyramid prevPyr_{};
    VirtualGyroEngine::FramePyramid currPyr_{};
    std::vector<VisPairSample> visPairs_;
    std::vector<PointPairFrame> pointFrames_;
};

} // namespace gf
