#pragma once

#include "CalibrationProfile.hpp"
#include "Quaternion.hpp"
#include "TrajectoryEngine.hpp"

#include <cstdint>
#include <vector>

namespace gf {

static constexpr int kRollingShutterSlices = 16;

struct GFWarpMatrixPacket {
    float m[9]; // Center-row 3x3 inverse homography H_inv
    float zoomUsed;
};

struct GFWarpFrameLUT {
    float sliceH[kRollingShutterSlices][9]; // 16-slice per-row inverse homography LUT
    float zoomUsed;
};

struct WarpComputationResult {
    std::vector<GFWarpMatrixPacket> packets;
    std::vector<GFWarpFrameLUT> frameLUTs;
    double minZoom = 1.0;
    double maxZoom = 1.0;
    double avgZoom = 1.0;
};

class FrameWarpCalculator {
public:
    static void computeRotationMatrix3x3(const Quat& q, double outR[9]);

    static void computeInverseHomography(
        const Quat& qCorr,
        double fx,
        double fy,
        double cx,
        double cy,
        double zoom,
        float outH[9]
    );

    static double computeRequiredZoom(
        const Quat& qCorr,
        uint32_t width,
        uint32_t height,
        double fx,
        double fy,
        double cx,
        double cy,
        double borderMarginPx = 4.0
    );

    static WarpComputationResult computeAllFrames(
        const std::vector<Quat>& rawQuats,
        const std::vector<Quat>& smoothQuats,
        const CalibrationProfile& profile,
        double nominalFps,
        double maxZoomLimit
    );

    static WarpComputationResult computeAllFramesWithRollingShutter(
        const TrajectoryEngine& engine,
        const TrajectoryResult& traj,
        const CalibrationProfile& profile,
        double nominalFps,
        double maxZoomLimit,
        bool rollingShutterCorrection,
        bool dynamicZoomEnabled
    );
};

} // namespace gf
