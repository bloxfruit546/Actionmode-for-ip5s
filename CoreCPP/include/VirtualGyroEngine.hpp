#pragma once

#include "SidecarFormat.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace gf {

struct VirtualGyroConfig {
    uint32_t videoWidth = 1920;
    uint32_t videoHeight = 1080;
    double nominalFps = 30.0;
    double readoutTimeSec = 0.0152;
    double lensFx = 1660.0;
    double lensFy = 1660.0;
    double lensCx = 960.0;
    double lensCy = 540.0;
    double distK1 = 0.0;
    double distK2 = 0.0;
    double distP1 = 0.0;
    double distP2 = 0.0;
    std::string uuid;
};

struct VirtualGyroResult {
    bool success = false;
    uint64_t frameCount = 0;
    uint64_t gyroCount = 0;
    double meanInlierRatio = 0.0;
    std::string errorMessage;
};

/**
 * Streaming 2-frame-in-RAM Virtual Gyro generator using 3-level Pyramidal Lucas-Kanade
 * with Forward-Backward consistency (< 1.0 px) and 80-iteration RANSAC 3-DOF rotational
 * homography to reject moving foreground objects.
 */
class VirtualGyroEngine {
public:
    struct PyramidLevel {
        int width = 0;
        int height = 0;
        std::vector<uint8_t> data;
    };

    struct FramePyramid {
        double timestampSec = 0.0;
        PyramidLevel levels[3];
    };

    struct VisualOmegaSample {
        double timestampSec = 0.0;
        double wxCam = 0.0;
        double wyCam = 0.0;
        double wzCam = 0.0;
    };

    struct TrackedPointPair {
        double u0 = 0.0;
        double v0 = 0.0;
        double u1 = 0.0;
        double v1 = 0.0;
    };

    explicit VirtualGyroEngine(const VirtualGyroConfig& config);

    bool pushLumaFrame(const uint8_t* luma, int width, int height, double timestampSec);
    VirtualGyroResult finalizeAndWrite(const std::string& outputSidecarPath);

    const std::vector<GFFrameRecord>& frames() const { return frames_; }
    const std::vector<VisualOmegaSample>& estimatedSamples() const { return gyroSamples_; }
    const std::vector<double>& sampleInlierRatios() const { return inlierRatios_; }
    const std::vector<int>& sampleInlierCounts() const { return inlierCounts_; }
    double meanInlierRatio() const {
        return (inlierRatioCount_ > 0) ? (inlierRatioSum_ / static_cast<double>(inlierRatioCount_)) : 0.0;
    }

    static void buildPyramid(const uint8_t* luma, int width, int height, double ts, FramePyramid& outPyr);
    static bool estimateRotationBetweenPyramids(
        const FramePyramid& prev,
        const FramePyramid& curr,
        const VirtualGyroConfig& config,
        VisualOmegaSample& outSample,
        double& outInlierRatio,
        int* outInlierCount = nullptr,
        std::vector<TrackedPointPair>* outInlierPoints = nullptr
    );

private:
    bool trackAndSolveRotation(
        const FramePyramid& prev,
        const FramePyramid& curr,
        VisualOmegaSample& outSample,
        double& outInlierRatio,
        int* outInlierCount = nullptr,
        std::vector<TrackedPointPair>* outInlierPoints = nullptr
    ) const;

    VirtualGyroConfig config_;
    bool hasPrevFrame_ = false;
    FramePyramid prevPyr_;
    FramePyramid currPyr_;
    std::vector<GFFrameRecord> frames_;
    std::vector<VisualOmegaSample> gyroSamples_;
    std::vector<double> inlierRatios_;
    std::vector<int> inlierCounts_;
    double inlierRatioSum_ = 0.0;
    size_t inlierRatioCount_ = 0;
};

} // namespace gf
