#pragma once

#include "CalibrationProfile.hpp"
#include "Quaternion.hpp"

#include <cstdint>
#include <vector>

namespace gf {

struct GFWarpMatrixPacket {
    float m[9]; // Row-major 3x3 inverse homography H_inv: dst pixel (u,v,1) -> src pixel (u_s, v_s, w_s)
    float zoomUsed;
};

struct WarpComputationResult {
    std::vector<GFWarpMatrixPacket> packets;
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
};

} // namespace gf
