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
    float outH[9],
    double shiftXBase,
    double shiftYBase
) {
    double R[9];
    computeRotationMatrix3x3(qCorr, R);

    const double invFxZ = 1.0 / (fx * zoom);
    const double invFyZ = 1.0 / (fy * zoom);
    const double tx = -cx * invFxZ;
    const double ty = -cy * invFyZ;

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

    double H0 = fx * B[0] + cx * B[6];
    double H1 = fx * B[1] + cx * B[7];
    double H2 = fx * B[2] + cx * B[8];

    double H3 = fy * B[3] + cy * B[6];
    double H4 = fy * B[4] + cy * B[7];
    double H5 = fy * B[5] + cy * B[8];

    double H6 = B[6];
    double H7 = B[7];
    double H8 = B[8];

    // Planar 2D translation in sensor pixel space:
    // u_sensor' = u_sensor + shiftXBase, v_sensor' = v_sensor + shiftYBase
    // Exact shift with ZERO projective distortion and ZERO keystone tilt.
    if (std::fabs(shiftXBase) > 1e-6 || std::fabs(shiftYBase) > 1e-6) {
        H0 += shiftXBase * H6;
        H1 += shiftXBase * H7;
        H2 += shiftXBase * H8;

        H3 += shiftYBase * H6;
        H4 += shiftYBase * H7;
        H5 += shiftYBase * H8;
    }

    outH[0] = static_cast<float>(H0);
    outH[1] = static_cast<float>(H1);
    outH[2] = static_cast<float>(H2);

    outH[3] = static_cast<float>(H3);
    outH[4] = static_cast<float>(H4);
    outH[5] = static_cast<float>(H5);

    outH[6] = static_cast<float>(H6);
    outH[7] = static_cast<float>(H7);
    outH[8] = static_cast<float>(H8);
}

double FrameWarpCalculator::computeRequiredZoom(
    const Quat& qCorr,
    uint32_t width,
    uint32_t height,
    double fx,
    double fy,
    double cx,
    double cy,
    double borderMarginPx,
    double cxVirtOffset,
    double cyVirtOffset
) {
    const double maxU = static_cast<double>(width - 1);
    const double maxV = static_cast<double>(height - 1);

    std::array<std::pair<double, double>, 16> boundaryPts{};
    int idx = 0;
    for (int k = 0; k < 4; ++k) {
        const double t = static_cast<double>(k) / 4.0;
        boundaryPts[idx++] = {t * maxU, 0.0};
        boundaryPts[idx++] = {maxU, t * maxV};
        boundaryPts[idx++] = {(1.0 - t) * maxU, maxV};
        boundaryPts[idx++] = {0.0, (1.0 - t) * maxV};
    }

    const double minValidX = borderMarginPx;
    const double maxValidX = maxU - borderMarginPx;
    const double minValidY = borderMarginPx;
    const double maxValidY = maxV - borderMarginPx;

    auto fitsInside = [&](double z) -> bool {
        float H[9];
        computeInverseHomography(qCorr, fx, fy, cx, cy, z, H, cxVirtOffset, cyVirtOffset);
        for (const auto& pt : boundaryPts) {
            const double u = pt.first;
            const double v = pt.second;
            const double w = static_cast<double>(H[6]) * u + static_cast<double>(H[7]) * v + static_cast<double>(H[8]);
            if (w <= 0.25) {
                return false;
            }
            const double us = (static_cast<double>(H[0]) * u + static_cast<double>(H[1]) * v + static_cast<double>(H[2])) / w;
            const double vs = (static_cast<double>(H[3]) * u + static_cast<double>(H[4]) * v + static_cast<double>(H[5])) / w;
            if (us < minValidX || us > maxValidX || vs < minValidY || vs > maxValidY) {
                return false;
            }
            const double xn = (us - cx) / fx;
            const double yn = (vs - cy) / fy;
            if ((xn * xn + yn * yn) >= 2.2) {
                return false;
            }
        }
        return true;
    };

    if (fitsInside(1.0)) {
        return 1.0;
    }

    double lo = 1.0;
    double hi = 4.0;
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
    TrajectoryEngine dummyEngine;
    TrajectoryResult traj;
    traj.rawFrameQuats = rawQuats;
    traj.smoothFrameQuats = smoothQuats;
    traj.frameTimestampsSec.resize(rawQuats.size(), 0.0);
    traj.valid = !rawQuats.empty();
    return computeAllFramesWithRollingShutter(
        dummyEngine,
        traj,
        profile,
        nominalFps,
        maxZoomLimit,
        false,
        true
    );
}

WarpComputationResult FrameWarpCalculator::computeAllFramesWithRollingShutter(
    const TrajectoryEngine& engine,
    const TrajectoryResult& traj,
    const CalibrationProfile& profile,
    double nominalFps,
    double maxZoomLimit,
    bool rollingShutterCorrection,
    bool dynamicZoomEnabled,
    const std::vector<double>* residualShiftX,
    const std::vector<double>* residualShiftY,
    EdgeFillMode edgeFillMode,
    double fovScaleBoost
) {
    WarpComputationResult res;
    const size_t n = std::min(traj.rawFrameQuats.size(), traj.smoothFrameQuats.size());
    if (n == 0) {
        return res;
    }

    const bool hasResidual = (residualShiftX != nullptr && residualShiftY != nullptr &&
                              residualShiftX->size() == n && residualShiftY->size() == n);
    auto getShift = [&](size_t idx) -> std::pair<double, double> {
        if (!hasResidual) {
            return {0.0, 0.0};
        }
        return {(*residualShiftX)[idx], (*residualShiftY)[idx]};
    };

    const double dt = (nominalFps > 1.0) ? (1.0 / nominalFps) : (1.0 / 30.0);

    // Step 1: Smooth Iterative Trajectory Pull-back if maxZoomLimit > 1.0
    // Prevents hard-clamp jerk while respecting maxZoomLimit.
    std::vector<Quat> adjustedSmooth = traj.smoothFrameQuats;
    if (maxZoomLimit > 1.0) {
        std::vector<double> pullFactor(n, 1.0);
        for (size_t i = 0; i < n; ++i) {
            const Quat qCorrFull = (traj.rawFrameQuats[i].conjugate() * traj.smoothFrameQuats[i]).normalized();
            const double zReq = computeRequiredZoom(
                qCorrFull, profile.width, profile.height,
                profile.fx, profile.fy, profile.cx, profile.cy, 4.0
            );
            if (zReq > maxZoomLimit) {
                double lo = 0.0;
                double hi = 1.0;
                for (int iter = 0; iter < 12; ++iter) {
                    const double mid = 0.5 * (lo + hi);
                    const Quat qTest = Quat::slerp(traj.rawFrameQuats[i], traj.smoothFrameQuats[i], mid);
                    const Quat qCorrTest = (traj.rawFrameQuats[i].conjugate() * qTest).normalized();
                    const double zTest = computeRequiredZoom(
                        qCorrTest, profile.width, profile.height,
                        profile.fx, profile.fy, profile.cx, profile.cy, 4.0
                    );
                    if (zTest <= maxZoomLimit) {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                pullFactor[i] = lo;
            }
        }

        // Smooth pullFactor (two-pass min-envelope IIR) so pull-back never causes derivative discontinuities
        const double alphaPull = std::exp(-dt / 0.35);
        for (size_t i = 1; i < n; ++i) {
            pullFactor[i] = std::min(pullFactor[i], alphaPull * pullFactor[i - 1] + (1.0 - alphaPull) * pullFactor[i]);
        }
        for (size_t i = n - 1; i > 0; --i) {
            const size_t idx = i - 1;
            pullFactor[idx] = std::min(pullFactor[idx], alphaPull * pullFactor[idx + 1] + (1.0 - alphaPull) * pullFactor[idx]);
        }

        for (size_t i = 0; i < n; ++i) {
            if (pullFactor[i] < 0.9999) {
                adjustedSmooth[i] = Quat::slerp(traj.rawFrameQuats[i], traj.smoothFrameQuats[i], pullFactor[i]);
            }
        }
    }

    const bool useRS = rollingShutterCorrection && (std::fabs(profile.readoutTimeSec) > 1e-5) && (traj.frameTimestampsSec.size() == n);

    // Step 2: Compute minimum safe zoom per frame across slice 0, mid, and 63 including 2D translation shifts
    std::vector<double> zMin(n, 1.0);
    for (size_t i = 0; i < n; ++i) {
        const auto shift = hasResidual ? getShift(i) : std::make_pair(0.0, 0.0);
        const Quat qCorrMid = (traj.rawFrameQuats[i].conjugate() * adjustedSmooth[i]).normalized();
        double zReq = computeRequiredZoom(
            qCorrMid, profile.width, profile.height,
            profile.fx, profile.fy, profile.cx, profile.cy, 4.0,
            shift.first, shift.second
        );
        if (useRS) {
            const Quat qTop = engine.sampleRawQuaternionAt(traj.frameTimestampsSec[i] - 0.5 * profile.readoutTimeSec);
            const Quat qBot = engine.sampleRawQuaternionAt(traj.frameTimestampsSec[i] + 0.5 * profile.readoutTimeSec);
            const Quat qCorrTop = (qTop.conjugate() * adjustedSmooth[i]).normalized();
            const Quat qCorrBot = (qBot.conjugate() * adjustedSmooth[i]).normalized();
            zReq = std::max(zReq, computeRequiredZoom(qCorrTop, profile.width, profile.height, profile.fx, profile.fy, profile.cx, profile.cy, 4.0, shift.first, shift.second));
            zReq = std::max(zReq, computeRequiredZoom(qCorrBot, profile.width, profile.height, profile.fx, profile.fy, profile.cx, profile.cy, 4.0, shift.first, shift.second));
        }
        zMin[i] = zReq;
    }

    // Step 3: Cinema Anti-Spike Dynamic Zoom Controller:
    //   1. Peak-Hold Plateau (0.65s): holds any zoom peak flat for >= 0.65s (zero 125ms snap!)
    //   2. Ultra-Slow Release (tau=2.5s, max 0.08x/sec): imperceptible zoom-out
    //   3. Finite Cosine Pre-Attack (0.30s): smooth ramp into turns with zero early intro bleed
    std::vector<double> zSmooth = zMin;
    if (dynamicZoomEnabled) {
        const double effFps = (nominalFps > 1.0) ? nominalFps : 30.0;
        const int holdFrames = std::max(4, static_cast<int>(std::round(0.65 * effFps)));
        const int attackFrames = std::max(3, static_cast<int>(std::round(0.30 * effFps)));
        constexpr double kPi = 3.14159265358979323846;

        // 3.1 Peak-Hold Plateau (backward window of 0.65s keeps peak flat after reaching it)
        std::vector<double> zHold(n, 1.0);
        for (size_t i = 0; i < n; ++i) {
            const int kLo = std::max(0, static_cast<int>(i) - holdFrames);
            double peak = zMin[i];
            for (int k = kLo; k <= static_cast<int>(i); ++k) {
                peak = std::max(peak, zMin[static_cast<size_t>(k)]);
            }
            zHold[i] = peak;
        }

        // 3.2 Ultra-Slow Release (tau = 2.5s, hard rate cap <= 0.08x / sec)
        const double alphaRel = std::exp(-dt / 2.50);
        const double maxDropPerFrame = 0.08 * dt;
        std::vector<double> zRel(n, 1.0);
        zRel[0] = zHold[0];
        for (size_t i = 1; i < n; ++i) {
            if (zHold[i] >= zRel[i - 1]) {
                zRel[i] = zHold[i];
            } else {
                const double expTarget = alphaRel * zRel[i - 1] + (1.0 - alphaRel) * zHold[i];
                const double rateCapped = std::max(zHold[i], zRel[i - 1] - maxDropPerFrame);
                zRel[i] = std::max(zHold[i], std::max(expTarget, rateCapped));
            }
        }

        // 3.3 Finite Cosine Pre-Attack (0.30s lookahead S-curve, strictly 0 bleed beyond 0.30s)
        zSmooth = zRel;
        for (size_t i = 0; i < n; ++i) {
            const int kHi = std::min(static_cast<int>(n) - 1, static_cast<int>(i) + attackFrames);
            double zMax = zRel[i];
            for (int k = static_cast<int>(i) + 1; k <= kHi; ++k) {
                const double u = static_cast<double>(k - static_cast<int>(i)) / static_cast<double>(attackFrames + 1);
                const double w = 0.5 * (1.0 + std::cos(kPi * u));
                const double cand = 1.0 + w * (zRel[static_cast<size_t>(k)] - 1.0);
                zMax = std::max(zMax, cand);
            }
            zSmooth[i] = std::max(zMin[i], zMax);
        }

        // Light 3-tap C1 polish strictly >= zMin[i]
        std::vector<double> zPolished = zSmooth;
        for (size_t i = 1; i + 1 < n; ++i) {
            const double val = 0.25 * zSmooth[i - 1] + 0.50 * zSmooth[i] + 0.25 * zSmooth[i + 1];
            zPolished[i] = std::max(zMin[i], val);
        }
        zSmooth = std::move(zPolished);
    } else {
        double staticMax = 1.0;
        for (double z : zMin) {
            staticMax = std::max(staticMax, z);
        }
        std::fill(zSmooth.begin(), zSmooth.end(), staticMax);
    }

    res.packets.resize(n);
    res.frameLUTs.resize(n);
    res.minZoom = 1e9;
    res.maxZoom = 0.0;
    double sumZoom = 0.0;

    const double boost = (edgeFillMode != EdgeFillMode::StandardCrop)
        ? std::min(1.0, std::max(0.0, fovScaleBoost))
        : 0.0;

    // Step 4: Fixed static margin for 2D translational polish (zero scale2D oscillation)
    for (size_t i = 0; i < n; ++i) {
        const std::pair<double, double> shift = hasResidual
            ? getShift(i)
            : std::make_pair(0.0, 0.0);
        double zFinal = zSmooth[i];
        if (maxZoomLimit > 1.0) {
            zFinal = std::min(zFinal, maxZoomLimit);
        }
        if (boost > 0.0) {
            zFinal = 1.0 + (1.0 - boost) * (zFinal - 1.0);
        }

        const Quat qCorrMid = (traj.rawFrameQuats[i].conjugate() * adjustedSmooth[i]).normalized();
        GFWarpMatrixPacket pkt{};
        computeInverseHomography(
            qCorrMid, profile.fx, profile.fy, profile.cx, profile.cy, zFinal, pkt.m, shift.first, shift.second
        );
        pkt.zoomUsed = static_cast<float>(zFinal);
        res.packets[i] = pkt;

        GFWarpFrameLUT lut{};
        lut.zoomUsed = static_cast<float>(zFinal);
        for (int s = 0; s < kRollingShutterSlices; ++s) {
            if (useRS) {
                const double frac = (static_cast<double>(s) / static_cast<double>(kRollingShutterSlices - 1)) - 0.5;
                const double tRow = traj.frameTimestampsSec[i] + frac * profile.readoutTimeSec;
                const Quat qRawRow = engine.sampleRawQuaternionAt(tRow);
                const Quat qCorrRow = (qRawRow.conjugate() * adjustedSmooth[i]).normalized();
                computeInverseHomography(
                    qCorrRow, profile.fx, profile.fy, profile.cx, profile.cy, zFinal, lut.sliceH[s], shift.first, shift.second
                );
            } else {
                std::copy(std::begin(pkt.m), std::end(pkt.m), std::begin(lut.sliceH[s]));
            }
        }
        res.frameLUTs[i] = lut;

        res.minZoom = std::min(res.minZoom, zFinal);
        res.maxZoom = std::max(res.maxZoom, zFinal);
        sumZoom += zFinal;
    }

    res.adjustedSmoothQuats = std::move(adjustedSmooth);
    res.avgZoom = sumZoom / static_cast<double>(n);
    return res;
}

} // namespace gf
