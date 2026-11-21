#pragma once

#include "CalibrationProfile.hpp"
#include "TrajectoryEngine.hpp"

#include <cstdint>
#include <vector>

namespace gf {

struct Translational2DResult {
    bool valid = false;
    std::vector<double> shiftXBase; // Full-res shift in Zoom=1.0 space (px)
    std::vector<double> shiftYBase; // Full-res shift in Zoom=1.0 space (px)
    double maxShiftPx = 0.0;
    double rmsShiftPx = 0.0;
    size_t validFrameCount = 0;
};

/// Streaming zero-RAM (28.8 KB fixed footprint) 160x90 Gyro-Rectified 2D Translational Stabilizer.
/// Measures and removes 1.5-4 Hz footstep bounce and near-field parallax after 3D gyro rectification.
class TranslationalStabilizer2D {
public:
    void reset(const CalibrationProfile& profile, const TrajectoryResult& traj, double strength = 0.0);

    void feedLumaFrame(size_t frameIndex, const uint8_t* fullLuma, int srcW, int srcH, int srcStride);

    Translational2DResult finalize(double nominalFps) const;

    int rectWidth() const { return rectW_; }
    int rectHeight() const { return rectH_; }

private:
    CalibrationProfile profile_{};
    std::vector<Quat> rawQuats_;
    std::vector<Quat> smoothQuats_;
    double strength_ = 0.0;

    int rectW_ = 160;
    int rectH_ = 90;

    // Fixed 2-frame ring buffer: 2 * (160*90 luma + 160*90 mask) = 57.6 KB (28.8 KB luma)
    std::vector<uint8_t> scratchDown_;
    std::vector<uint8_t> prevRect_;
    std::vector<uint8_t> currRect_;
    std::vector<uint8_t> prevMask_;
    std::vector<uint8_t> currMask_;
    bool hasPrev_ = false;

    struct FrameDelta {
        double dx160 = 0.0;
        double dy160 = 0.0;
        double confidence = 0.0;
    };
    std::vector<FrameDelta> deltas_;
};

} // namespace gf
