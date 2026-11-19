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
    double cxVirtOffset,
    double cyVirtOffset
) {
    double R[9];
    computeRotationMatrix3x3(qCorr, R);

    const double invFxZ = 1.0 / (fx * zoom);
    const double invFyZ = 1.0 / (fy * zoom);
    const double tx = -(cx + cxVirtOffset) * invFxZ;
    const double ty = -(cy + cyVirtOffset) * invFyZ;

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

    // Step 2: Compute minimum safe zoom per frame across slice 0, mid, and 31
    std::vector<double> zMin(n, 1.0);
    for (size_t i = 0; i < n; ++i) {
        const auto shift = getShift(i);
        const Quat qCorrMid = (traj.rawFrameQuats[i].conjugate() * adjustedSmooth[i]).normalized();
        double zReq = computeRequiredZoom(
            qCorrMid, profile.width, profile.height,
            profile.fx, profile.fy, profile.cx, profile.cy, 4.0, shift.first, shift.second
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

    // Step 3: Tight Anti-Pumping Dynamic Zoom Ramp (Finite 0.18s Hermite Lookahead + 0.28s Release + 0.03x Deadband)
    // Eliminates early zoom bleed during stationary intro while guaranteeing 0% black borders.
    std::vector<double> zSmooth = zMin;
    if (dynamicZoomEnabled) {
        const double effFps = (nominalFps > 1.0) ? nominalFps : 30.0;
        const int lookaheadFrames = std::max(2, static_cast<int>(std::round(0.18 * effFps)));
        const int releaseFrames = std::max(3, static_cast<int>(std::round(0.28 * effFps)));

        std::vector<double> zEnv = zMin;
        for (size_t i = 0; i < n; ++i) {
            const int kLo = std::max(0, static_cast<int>(i) - releaseFrames);
            const int kHi = std::min(static_cast<int>(n) - 1, static_cast<int>(i) + lookaheadFrames);
            double zMax = zMin[i];
            for (int k = kLo; k <= kHi; ++k) {
                if (k == static_cast<int>(i)) continue;
                const double dist = std::fabs(static_cast<double>(k - static_cast<int>(i)));
                const double span = (k > static_cast<int>(i))
                    ? static_cast<double>(lookaheadFrames + 1)
                    : static_cast<double>(releaseFrames + 1);
                const double u = std::min(1.0, dist / span);
                const double w = 1.0 - (u * u * (3.0 - 2.0 * u)); // Cubic Hermite smoothstep decay
                const double cand = 1.0 + w * (zMin[static_cast<size_t>(k)] - 1.0);
                zMax = std::max(zMax, cand);
            }
            zEnv[i] = zMax;
        }

        // 0.03x hysteresis deadband prevents micro-breathing on small fluctuations
        double zHold = zEnv[0];
        std::vector<double> zDead(n, zHold);
        for (size_t i = 1; i < n; ++i) {
            if (zEnv[i] > zHold) {
                zHold = zEnv[i];
            } else if (zEnv[i] < zHold - 0.03) {
                zHold = zEnv[i] + 0.03;
            }
            zDead[i] = std::max(zMin[i], zHold);
        }

        // Light 3-tap C1 polish strictly >= zMin[i]
        zSmooth = zDead;
        for (size_t i = 1; i + 1 < n; ++i) {
            const double val = 0.25 * zDead[i - 1] + 0.50 * zDead[i] + 0.25 * zDead[i + 1];
            zSmooth[i] = std::max(zMin[i], val);
        }
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

    for (size_t i = 0; i < n; ++i) {
        const auto shift = getShift(i);
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
