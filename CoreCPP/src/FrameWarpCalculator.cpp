#include "FrameWarpCalculator.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace gf {

void FrameWarpCalculator::computeRotationMatrix3x3(const Quat& qIn, double outR[9]) {
    const Quat q = qIn.normalized();
    const double w = q.w, x = q.x, y = q.y, z = q.z;

    const double xx = x * x, yy = y * y, zz = z * z;
    const double xy = x * y, xz = x * z, yz = y * z;
    const double wx = w * x, wy = w * y, wz = w * z;

    outR[0] = 1.0 - 2.0 * (yy + zz);
    outR[1] = 2.0 * (xy - wz);
    outR[2] = 2.0 * (xz + wy);

    outR[3] = 2.0 * (xy + wz);
    outR[4] = 1.0 - 2.0 * (xx + zz);
    outR[5] = 2.0 * (yz - wx);

    outR[6] = 2.0 * (xz - wy);
    outR[7] = 2.0 * (yz + wx);
    outR[8] = 1.0 - 2.0 * (xx + yy);
}

void FrameWarpCalculator::computeInverseHomography(
    const Quat& qCorr,
    double fx,
    double fy,
    double cx,
    double cy,
    double zoom,
    float outH[9]
) {
    double R[9];
    computeRotationMatrix3x3(qCorr, R);

    const double invFxZ = 1.0 / (fx * zoom);
    const double invFyZ = 1.0 / (fy * zoom);
    const double tx = -cx * invFxZ;
    const double ty = -cy * invFyZ;

    // B = R * K_virt(z)^(-1)
    double B[9];
    B[0] = R[0] * invFxZ;
    B[1] = R[1] * invFyZ;
    B[2] = R[0] * tx + R[1] * ty + R[2];

    B[3] = R[3] * invFxZ;
    B[4] = R[4] * invFyZ;
    B[5] = R[3] * tx + R[4] * ty + R[5];

    B[6] = R[6] * invFxZ;
    B[7] = R[7] * invFyZ;
    B[8] = R[6] * tx + R[7] * ty + R[8];

    // H = K_real * B
    outH[0] = static_cast<float>(fx * B[0] + cx * B[6]);
    outH[1] = static_cast<float>(fx * B[1] + cx * B[7]);
    outH[2] = static_cast<float>(fx * B[2] + cx * B[8]);

    outH[3] = static_cast<float>(fy * B[3] + cy * B[6]);
    outH[4] = static_cast<float>(fy * B[4] + cy * B[7]);
    outH[5] = static_cast<float>(fy * B[5] + cy * B[8]);

    outH[6] = static_cast<float>(B[6]);
    outH[7] = static_cast<float>(B[7]);
    outH[8] = static_cast<float>(B[8]);
}

double FrameWarpCalculator::computeRequiredZoom(
    const Quat& qCorr,
    uint32_t width,
    uint32_t height,
    double fx,
    double fy,
    double cx,
    double cy,
    double borderMarginPx
) {
    const double maxU = static_cast<double>(width - 1);
    const double maxV = static_cast<double>(height - 1);

    // 16 boundary points along the 4 edges
    std::array<std::pair<double, double>, 16> boundaryPts{};
    int idx = 0;
    for (int k = 0; k < 4; ++k) {
        const double t = static_cast<double>(k) / 4.0;
        boundaryPts[idx++] = {t * maxU, 0.0};                 // Top edge
        boundaryPts[idx++] = {maxU, t * maxV};                // Right edge
        boundaryPts[idx++] = {(1.0 - t) * maxU, maxV};        // Bottom edge
        boundaryPts[idx++] = {0.0, (1.0 - t) * maxV};         // Left edge
    }

    const double minValidX = borderMarginPx;
    const double maxValidX = maxU - borderMarginPx;
    const double minValidY = borderMarginPx;
    const double maxValidY = maxV - borderMarginPx;

    auto fitsInside = [&](double z) -> bool {
        float H[9];
        computeInverseHomography(qCorr, fx, fy, cx, cy, z, H);
        for (const auto& pt : boundaryPts) {
            const double u = pt.first;
            const double v = pt.second;
            const double w = static_cast<double>(H[6]) * u + static_cast<double>(H[7]) * v + static_cast<double>(H[8]);
            if (w <= 1e-4) {
                return false;
            }
            const double us = (static_cast<double>(H[0]) * u + static_cast<double>(H[1]) * v + static_cast<double>(H[2])) / w;
            const double vs = (static_cast<double>(H[3]) * u + static_cast<double>(H[4]) * v + static_cast<double>(H[5])) / w;
            if (us < minValidX || us > maxValidX || vs < minValidY || vs > maxValidY) {
                return false;
            }
        }
        return true;
    };

    if (fitsInside(1.0)) {
        return 1.0;
    }

    double lo = 1.0;
    double hi = 3.0;
    if (!fitsInside(hi)) {
        return hi;
    }

    for (int iter = 0; iter < 18; ++iter) {
        const double mid = 0.5 * (lo + hi);
        if (fitsInside(mid)) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    return hi;
}

WarpComputationResult FrameWarpCalculator::computeAllFrames(
    const std::vector<Quat>& rawQuats,
    const std::vector<Quat>& smoothQuats,
    const CalibrationProfile& profile,
    double nominalFps,
    double maxZoomLimit
) {
    WarpComputationResult res;
    const size_t n = std::min(rawQuats.size(), smoothQuats.size());
    if (n == 0) {
        return res;
    }

    std::vector<Quat> qCorrs(n);
    std::vector<double> zMin(n, 1.0);
    for (size_t i = 0; i < n; ++i) {
        qCorrs[i] = (rawQuats[i].conjugate() * smoothQuats[i]).normalized();
        zMin[i] = computeRequiredZoom(
            qCorrs[i],
            profile.width,
            profile.height,
            profile.fx,
            profile.fy,
            profile.cx,
            profile.cy,
            4.0
        );
    }

    // Smooth upper-envelope zoom curve (zero-phase two-pass IIR, clamped >= zMin[i])
    const double dt = (nominalFps > 1.0) ? (1.0 / nominalFps) : (1.0 / 30.0);
    const double alphaZoom = std::exp(-dt / 0.5);
    std::vector<double> zFwd = zMin;
    for (size_t i = 1; i < n; ++i) {
        zFwd[i] = std::max(zMin[i], alphaZoom * zFwd[i - 1] + (1.0 - alphaZoom) * zMin[i]);
    }
    std::vector<double> zSmooth = zFwd;
    for (size_t i = n - 1; i > 0; --i) {
        const size_t idx = i - 1;
        zSmooth[idx] = std::max(zMin[idx], alphaZoom * zSmooth[idx + 1] + (1.0 - alphaZoom) * zFwd[idx]);
    }

    res.packets.resize(n);
    res.minZoom = 1e9;
    res.maxZoom = 0.0;
    double sumZoom = 0.0;

    for (size_t i = 0; i < n; ++i) {
        double zFinal = zSmooth[i];
        if (maxZoomLimit > 1.0) {
            zFinal = std::min(zFinal, maxZoomLimit);
        }

        GFWarpMatrixPacket pkt{};
        computeInverseHomography(
            qCorrs[i],
            profile.fx,
            profile.fy,
            profile.cx,
            profile.cy,
            zFinal,
            pkt.m
        );
        pkt.zoomUsed = static_cast<float>(zFinal);
        res.packets[i] = pkt;

        res.minZoom = std::min(res.minZoom, zFinal);
        res.maxZoom = std::max(res.maxZoom, zFinal);
        sumZoom += zFinal;
    }

    res.avgZoom = sumZoom / static_cast<double>(n);
    return res;
}

} // namespace gf
