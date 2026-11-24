#include "ConstrainedPathOptimizer.hpp"
#include "FrameWarpCalculator.hpp"

#include <algorithm>
#include <cmath>

namespace gf {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kRad2Deg = 180.0 / kPi;

Vec3 quatLogVec(const Quat& qIn) {
    Quat q = qIn.normalized();
    if (q.w < 0.0) {
        q = Quat(-q.w, -q.x, -q.y, -q.z);
    }
    const double vNorm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    if (vNorm < 1e-8) {
        return Vec3(2.0 * q.x, 2.0 * q.y, 2.0 * q.z);
    }
    const double theta = 2.0 * std::atan2(vNorm, q.w);
    const double scale = theta / vNorm;
    return Vec3(q.x * scale, q.y * scale, q.z * scale);
}

Quat quatExpVec(const Vec3& r) {
    const double theta = r.norm();
    if (theta < 1e-8) {
        return Quat(1.0, 0.5 * r.x, 0.5 * r.y, 0.5 * r.z).normalized();
    }
    const double half = 0.5 * theta;
    const double s = std::sin(half) / theta;
    return Quat(std::cos(half), r.x * s, r.y * s, r.z * s).normalized();
}

Quat quatFromBasis(const Vec3& right, const Vec3& down, const Vec3& forward) {
    const double m00 = right.x, m01 = down.x, m02 = forward.x;
    const double m10 = right.y, m11 = down.y, m12 = forward.y;
    const double m20 = right.z, m21 = down.z, m22 = forward.z;

    const double trace = m00 + m11 + m22;
    Quat q;
    if (trace > 0.0) {
        const double s = 2.0 * std::sqrt(trace + 1.0);
        q.w = 0.25 * s;
        q.x = (m21 - m12) / s;
        q.y = (m02 - m20) / s;
        q.z = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        const double s = 2.0 * std::sqrt(1.0 + m00 - m11 - m22);
        q.w = (m21 - m12) / s;
        q.x = 0.25 * s;
        q.y = (m01 + m10) / s;
        q.z = (m02 + m20) / s;
    } else if (m11 > m22) {
        const double s = 2.0 * std::sqrt(1.0 + m11 - m00 - m22);
        q.w = (m02 - m20) / s;
        q.x = (m01 + m10) / s;
        q.y = 0.25 * s;
        q.z = (m12 + m21) / s;
    } else {
        const double s = 2.0 * std::sqrt(1.0 + m22 - m00 - m11);
        q.w = (m10 - m01) / s;
        q.x = (m02 + m20) / s;
        q.y = (m12 + m21) / s;
        q.z = 0.25 * s;
    }
    return q.normalized();
}

int detectGravityQuadrantDeg(const Vec3& gWorld) {
    if (std::fabs(gWorld.y) >= std::fabs(gWorld.x)) {
        return (gWorld.y >= 0.0) ? 0 : 180;
    }
    return (gWorld.x >= 0.0) ? 90 : 270;
}

void smoothScalarGaussianTwoPass(std::vector<double>& values, double sigmaFrames) {
    const size_t n = values.size();
    if (n < 3 || sigmaFrames <= 0.1) {
        return;
    }
    const double alpha = std::exp(-1.0 / sigmaFrames);
    for (int pass = 0; pass < 2; ++pass) {
        for (size_t i = 1; i < n; ++i) {
            values[i] = alpha * values[i - 1] + (1.0 - alpha) * values[i];
        }
        for (size_t i = n - 1; i > 0; --i) {
            const size_t idx = i - 1;
            values[idx] = alpha * values[idx + 1] + (1.0 - alpha) * values[idx];
        }
    }
}

Quat computeHorizonQuatForForward(
    const Quat& qRef,
    const Vec3& gWorldIn,
    const Quat& screenAlignQ
) {
    const Vec3 gW = gWorldIn.normalized();
    const Vec3 zc = qRef.rotateVector(Vec3(0.0, 0.0, 1.0)).normalized();
    Vec3 yc = gW - zc * gW.dot(zc);
    if (yc.normSquared() < 1e-6) {
        yc = qRef.rotateVector(Vec3(0.0, 1.0, 0.0));
    }
    yc = yc.normalized();
    const Vec3 xc = yc.cross(zc).normalized();
    return (quatFromBasis(xc, yc, zc) * screenAlignQ).normalized();
}

struct AxisParams {
    double smoothness = 0.78;
    double velocityDamping = 1.0;
    double fps = 30.0;
    bool suppressPanRelease = false;
    double zoomEconomyWeight = 0.0;
    double smoothInputScale = 1.0;
};

void computeSmoothnessWeights(
    double smoothness,
    double& outAlphaHold,
    double& outBetaBase,
    double& outGammaBase,
    double& outLambdaAnchor
) {
    const double s = std::min(1.0, std::max(0.0, smoothness));
    if (s <= 1e-4) {
        outAlphaHold = 0.0;
        outBetaBase = 0.0;
        outGammaBase = 0.0;
        outLambdaAnchor = 1.0;
        return;
    }
    const double boost = (s > 0.75) ? (2.5 * std::pow((s - 0.75) / 0.25, 1.4)) : 0.0;
    const double scale = 0.12 * s + 0.88 * s * s + boost;
    outAlphaHold = 1.25 * scale;
    outBetaBase = 0.75 * scale;
    outGammaBase = 0.15 * scale;
    outLambdaAnchor = std::max(0.25, 2.5 * (1.08 - 0.88 * s));
}

void factorBandedLDLT(
    const std::vector<double>& diag0,
    const std::vector<double>& diag1,
    const std::vector<double>& diag2,
    const std::vector<double>& diag3,
    double rho,
    std::vector<double>& invD,
    std::vector<double>& l1,
    std::vector<double>& l2,
    std::vector<double>& l3
) {
    const size_t n = diag0.size();
    invD.assign(n, 1.0);
    l1.assign(n, 0.0);
    l2.assign(n, 0.0);
    l3.assign(n, 0.0);
    std::vector<double> D(n, 1.0);

    for (size_t j = 0; j < n; ++j) {
        double dj = diag0[j] + rho;
        if (j >= 1) dj -= l1[j - 1] * l1[j - 1] * D[j - 1];
        if (j >= 2) dj -= l2[j - 2] * l2[j - 2] * D[j - 2];
        if (j >= 3) dj -= l3[j - 3] * l3[j - 3] * D[j - 3];
        if (!std::isfinite(dj) || dj < 1e-12) {
            dj = 1e-12;
        }
        D[j] = dj;
        const double inv = 1.0 / dj;
        invD[j] = inv;

        if (j + 1 < n) {
            double s1 = diag1[j];
            if (j >= 1) s1 -= l2[j - 1] * l1[j - 1] * D[j - 1];
            if (j >= 2) s1 -= l3[j - 2] * l2[j - 2] * D[j - 2];
            l1[j] = s1 * inv;
        }
        if (j + 2 < n) {
            double s2 = diag2[j];
            if (j >= 1) s2 -= l3[j - 1] * l1[j - 1] * D[j - 1];
            l2[j] = s2 * inv;
        }
        if (j + 3 < n) {
            l3[j] = diag3[j] * inv;
        }
    }
}

void solveFactoredBandedLDLT(
    const std::vector<double>& invD,
    const std::vector<double>& l1,
    const std::vector<double>& l2,
    const std::vector<double>& l3,
    const std::vector<double>& rhs,
    std::vector<double>& outX,
    std::vector<double>& yScratch
) {
    const size_t n = invD.size();
    yScratch.resize(n);
    for (size_t i = 0; i < n; ++i) {
        double sum = rhs[i];
        if (i >= 1) sum -= l1[i - 1] * yScratch[i - 1];
        if (i >= 2) sum -= l2[i - 2] * yScratch[i - 2];
        if (i >= 3) sum -= l3[i - 3] * yScratch[i - 3];
        yScratch[i] = sum;
    }
    for (size_t i = 0; i < n; ++i) {
        yScratch[i] *= invD[i];
    }
    outX.resize(n);
    for (size_t ii = n; ii > 0; --ii) {
        const size_t i = ii - 1;
        double sum = yScratch[i];
        if (i + 1 < n) sum -= l1[i] * outX[i + 1];
        if (i + 2 < n) sum -= l2[i] * outX[i + 2];
        if (i + 3 < n) sum -= l3[i] * outX[i + 3];
        outX[i] = sum;
    }
}

void solveAxis1DIRLS(
    const std::vector<double>& omegaBase,
    const std::vector<double>& lower,
    const std::vector<double>& upper,
    const std::vector<double>& panWeight,
    const AxisParams& params,
    std::vector<double>& uInOut
) {
    const size_t n = lower.size();
    if (n < 3) {
        return;
    }
    if (params.smoothness <= 1e-4 || params.smoothInputScale <= 1e-4) {
        for (size_t i = 0; i < n; ++i) {
            uInOut[i] = std::min(upper[i], std::max(lower[i], 0.0));
        }
        return;
    }

    const double fps = (params.fps > 1.0) ? params.fps : 30.0;
    const double dt = 1.0 / fps;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;

    double alphaHold = 0.0, betaBase = 0.0, gammaBase = 0.0, lambdaBase = 0.02;
    computeSmoothnessWeights(params.smoothness, alphaHold, betaBase, gammaBase, lambdaBase);

    const double panSuppression = params.suppressPanRelease ? 0.25 : 1.0;
    const double dampFactor = std::min(2.0, std::max(0.5, params.velocityDamping));

    std::vector<double> alphaArr(n, alphaHold);
    std::vector<double> betaArr(n, betaBase);
    std::vector<double> gammaArr(n, gammaBase);
    std::vector<double> lambdaArr(n, lambdaBase * dt);

    const double zoomEcon = params.zoomEconomyWeight;
    for (size_t i = 0; i < n; ++i) {
        const double p = std::min(1.0, std::max(0.0, panWeight[i] * panSuppression));
        const double alphaPanScale = std::max(0.002, (1.0 - 0.994 * p) / (1.0 + 0.5 * (dampFactor - 1.0) * p));
        alphaArr[i] = alphaHold * alphaPanScale * params.smoothInputScale;
        betaArr[i] = betaBase * (1.0 - 0.30 * p) * params.smoothInputScale;
        gammaArr[i] = gammaBase * (1.0 + 0.15 * p) * params.smoothInputScale;
        lambdaArr[i] = (lambdaBase + zoomEcon + 6.8 * p) * dt;
    }

    const size_t m1 = n - 1;
    const size_t m2 = (n >= 3) ? (n - 2) : 0;
    const size_t m3 = (n >= 4) ? (n - 3) : 0;

    std::vector<double> thetaBase(m1, 0.0);
    for (size_t i = 0; i < m1; ++i) {
        thetaBase[i] = omegaBase[i] * dt;
    }

    // Clamp initial warm-start inside box
    for (size_t i = 0; i < n; ++i) {
        uInOut[i] = std::min(upper[i], std::max(lower[i], uInOut[i]));
    }

    // Scale-invariant Charbonnier epsilons per step difference
    const double epsStep1 = std::max(0.008 * dt, 1.5e-4);
    const double epsStep2 = std::max(0.06 * dt2, 6.0e-5);
    const double epsStep3 = std::max(0.60 * dt3, 3.0e-5);

    std::vector<double> d0(n), d1(n), d2(n), d3(n), g(n);
    std::vector<double> invD(n), l1(n), l2(n), l3(n), yScratch(n), uCand(n);
    std::vector<double> admmRhs(n), zADMM(n, 0.0), yADMM(n, 0.0);
    bool hasAdmmWarmStart = false;
    double rhoPrev = 10.0;

    constexpr double s1[2] = {-1.0, 1.0};
    constexpr double s2[3] = {1.0, -2.0, 1.0};
    constexpr double s3[4] = {-1.0, 3.0, -3.0, 1.0};

    constexpr int kIirlsIters = 18;
    for (int irls = 0; irls < kIirlsIters; ++irls) {
        std::fill(d0.begin(), d0.end(), 0.0);
        std::fill(d1.begin(), d1.end(), 0.0);
        std::fill(d2.begin(), d2.end(), 0.0);
        std::fill(d3.begin(), d3.end(), 0.0);
        std::fill(g.begin(), g.end(), 0.0);

        auto stamp = [&](size_t baseIdx, int len, const double* s, double r, double w) {
            for (int a = 0; a < len; ++a) {
                const size_t ra = baseIdx + static_cast<size_t>(a);
                g[ra] -= w * s[a] * r;
                d0[ra] += w * s[a] * s[a];
                if (a + 1 < len) d1[ra] += w * s[a] * s[a + 1];
                if (a + 2 < len) d2[ra] += w * s[a] * s[a + 2];
                if (a + 3 < len) d3[ra] += w * s[a] * s[a + 3];
            }
        };

        // 1. Velocity L1 term: sum alpha_i * |d1_i|
        for (size_t i = 0; i < m1; ++i) {
            const double d1Val = thetaBase[i] + (uInOut[i + 1] - uInOut[i]);
            const double wv = alphaArr[i] / std::sqrt(d1Val * d1Val + epsStep1 * epsStep1);
            stamp(i, 2, s1, thetaBase[i], wv);
        }

        // 2. Acceleration L1 term: sum (beta_i / dt) * |d2_i|
        for (size_t i = 0; i < m2; ++i) {
            const double r2 = thetaBase[i + 1] - thetaBase[i];
            const double d2Val = r2 + (uInOut[i + 2] - 2.0 * uInOut[i + 1] + uInOut[i]);
            const double wa = (betaArr[i] / dt) / std::sqrt(d2Val * d2Val + epsStep2 * epsStep2);
            stamp(i, 3, s2, r2, wa);
        }

        // 3. Jerk L1 term: sum (gamma_i / dt^2) * |d3_i|
        for (size_t i = 0; i < m3; ++i) {
            const double r3 = thetaBase[i + 2] - 2.0 * thetaBase[i + 1] + thetaBase[i];
            const double d3Val = r3 + (uInOut[i + 3] - 3.0 * uInOut[i + 2] + 3.0 * uInOut[i + 1] - uInOut[i]);
            const double wj = (gammaArr[i] / dt2) / std::sqrt(d3Val * d3Val + epsStep3 * epsStep3);
            stamp(i, 4, s3, r3, wj);
        }

        // 4. Faithfulness anchor + shifted log-barrier cushion near bounds
        const double deltaB = 0.005;
        const double muB = 0.0008 * dt * params.smoothInputScale;
        double sumD0 = 0.0;
        for (size_t i = 0; i < n; ++i) {
            const double dLow = std::max(1e-4, (uInOut[i] - lower[i]) + deltaB);
            const double dHigh = std::max(1e-4, (upper[i] - uInOut[i]) + deltaB);
            const double gradB = muB * (1.0 / dHigh - 1.0 / dLow);
            const double hessB = muB * (1.0 / (dHigh * dHigh) + 1.0 / (dLow * dLow));
            d0[i] += lambdaArr[i] + hessB;
            g[i] += hessB * uInOut[i] - gradB;
            sumD0 += d0[i];
        }

        // 5A. Exact 1-Shot Direct Banded LDL^T Solve
        factorBandedLDLT(d0, d1, d2, d3, 0.0, invD, l1, l2, l3);
        solveFactoredBandedLDLT(invD, l1, l2, l3, g, uCand, yScratch);

        bool unconstrainedFeasible = true;
        for (size_t i = 0; i < n; ++i) {
            if (uCand[i] < lower[i] - 1e-9 || uCand[i] > upper[i] + 1e-9) {
                unconstrainedFeasible = false;
                break;
            }
        }
        if (unconstrainedFeasible) {
            for (size_t i = 0; i < n; ++i) {
                uInOut[i] = std::min(upper[i], std::max(lower[i], uCand[i]));
            }
            zADMM = uInOut;
            std::fill(yADMM.begin(), yADMM.end(), 0.0);
            hasAdmmWarmStart = true;
            continue;
        }

        // 5B. Frequency-Calibrated Over-Relaxed ADMM (well-conditioned rho <= 2500)
        const double meanD0 = sumD0 / static_cast<double>(n);
        double rho = std::min(2500.0, std::max(2.0, 5.0e-5 * meanD0));
        if (!hasAdmmWarmStart) {
            for (size_t i = 0; i < n; ++i) {
                zADMM[i] = std::min(upper[i], std::max(lower[i], uCand[i]));
                yADMM[i] = 0.0;
            }
            hasAdmmWarmStart = true;
        } else if (rhoPrev > 0.0) {
            const double ratio = rhoPrev / rho;
            for (size_t i = 0; i < n; ++i) {
                zADMM[i] = std::min(upper[i], std::max(lower[i], zADMM[i]));
                yADMM[i] *= ratio;
            }
        }
        rhoPrev = rho;
        factorBandedLDLT(d0, d1, d2, d3, rho, invD, l1, l2, l3);

        constexpr double kAlphaRel = 1.65;
        constexpr int kMaxAdmmIters = 40;
        const double primalTolSq = 1e-13 * static_cast<double>(n);
        const double dualTolSq = 1e-11 * static_cast<double>(n);

        for (int admm = 0; admm < kMaxAdmmIters; ++admm) {
            for (size_t i = 0; i < n; ++i) {
                admmRhs[i] = g[i] + rho * (zADMM[i] - yADMM[i]);
            }
            solveFactoredBandedLDLT(invD, l1, l2, l3, admmRhs, uCand, yScratch);

            double rPrimalSq = 0.0;
            double rDualSq = 0.0;
            for (size_t i = 0; i < n; ++i) {
                const double zOld = zADMM[i];
                const double uHat = kAlphaRel * uCand[i] - (kAlphaRel - 1.0) * zOld;
                const double zNew = std::min(upper[i], std::max(lower[i], uHat + yADMM[i]));
                yADMM[i] += uHat - zNew;
                zADMM[i] = zNew;
                const double dp = uCand[i] - zNew;
                const double dd = rho * (zNew - zOld);
                rPrimalSq += dp * dp;
                rDualSq += dd * dd;
            }

            if (rPrimalSq < primalTolSq && rDualSq < dualTolSq) {
                break;
            }

            if ((admm == 12 || admm == 26) && rPrimalSq > 0.0 && rDualSq > 0.0) {
                double factor = 1.0;
                if (rPrimalSq > 64.0 * rDualSq && rho < 5e4) {
                    factor = 2.5;
                } else if (rDualSq > 64.0 * rPrimalSq && rho > 0.2) {
                    factor = 0.4;
                }
                if (factor != 1.0) {
                    rho *= factor;
                    rhoPrev = rho;
                    const double yScale = 1.0 / factor;
                    for (size_t i = 0; i < n; ++i) {
                        yADMM[i] *= yScale;
                    }
                    factorBandedLDLT(d0, d1, d2, d3, rho, invD, l1, l2, l3);
                }
            }
        }

        uInOut = zADMM;
    }
}

void computePerAxisPanWeights(
    const std::vector<Vec3>& omegaBase,
    double fps,
    std::vector<double>& outPanX,
    std::vector<double>& outPanY,
    std::vector<double>& outPanZ,
    double& outAvgPanRatio
) {
    const size_t n = omegaBase.size() + 1;
    const double dt = (fps > 1.0) ? (1.0 / fps) : (1.0 / 30.0);
    const int radius = std::max(2, static_cast<int>(std::round(0.60 * fps)));

    outPanX.assign(n, 0.0);
    outPanY.assign(n, 0.0);
    outPanZ.assign(n, 0.0);

    std::vector<double> dx(n, 0.0), dy(n, 0.0), dz(n, 0.0), d3(n, 0.0);
    for (size_t i = 1; i < n; ++i) {
        const Vec3 step = omegaBase[i - 1] * dt;
        dx[i] = step.x;
        dy[i] = step.y;
        dz[i] = step.z;
        d3[i] = step.norm();
    }

    // Low-pass filter angular increments before measuring directional pan coherence so high-frequency shake
    // does not mask a constant-velocity pan underneath.
    std::vector<double> dxLPF = dx;
    std::vector<double> dyLPF = dy;
    std::vector<double> dzLPF = dz;
    const double lpfSigma = std::max(1.5, 0.22 * fps);
    smoothScalarGaussianTwoPass(dxLPF, lpfSigma);
    smoothScalarGaussianTwoPass(dyLPF, lpfSigma);
    smoothScalarGaussianTwoPass(dzLPF, lpfSigma);

    double panRatioSum = 0.0;
    auto axisPanActivity = [&](double netDisp, double pathLen, double dtSpan) -> double {
        const double dirRatio = std::fabs(netDisp) / (pathLen + 1e-4);
        const double rate = std::fabs(netDisp) / std::max(1e-4, dtSpan);
        if (dirRatio <= 0.72 || rate <= 0.055) {
            return 0.0;
        }
        const double rPart = std::min(1.0, (dirRatio - 0.72) / (0.92 - 0.72));
        const double vPart = std::min(1.0, (rate - 0.055) / (0.160 - 0.055));
        return rPart * vPart;
    };

    for (size_t i = 0; i < n; ++i) {
        const size_t lo = static_cast<size_t>(std::max(0, static_cast<int>(i) - radius));
        const size_t hi = static_cast<size_t>(std::min(static_cast<int>(n) - 1, static_cast<int>(i) + radius));
        double sumX = 0.0, lenX = 0.0;
        double sumY = 0.0, lenY = 0.0;
        double sumZ = 0.0, lenZ = 0.0;
        Vec3 net3(0.0, 0.0, 0.0);
        double len3 = 0.0;
        for (size_t k = lo + 1; k <= hi; ++k) {
            sumX += dxLPF[k]; lenX += std::fabs(dxLPF[k]);
            sumY += dyLPF[k]; lenY += std::fabs(dyLPF[k]);
            sumZ += dzLPF[k]; lenZ += std::fabs(dzLPF[k]);
            net3 = net3 + Vec3(dx[k], dy[k], dz[k]);
            len3 += d3[k];
        }
        const double dtSpan = static_cast<double>(hi - lo) * dt;
        panRatioSum += std::min(1.0, net3.norm() / (len3 + 1e-4));

        outPanX[i] = axisPanActivity(sumX, lenX, dtSpan);
        outPanY[i] = axisPanActivity(sumY, lenY, dtSpan);
        outPanZ[i] = axisPanActivity(sumZ, lenZ, dtSpan);
    }

    outAvgPanRatio = (n > 0) ? (panRatioSum / static_cast<double>(n)) : 0.0;
    const double sigma = std::max(1.5, 0.25 * fps);
    smoothScalarGaussianTwoPass(outPanX, sigma);
    smoothScalarGaussianTwoPass(outPanY, sigma);
    smoothScalarGaussianTwoPass(outPanZ, sigma);
}

} // namespace

bool ConstrainedPathOptimizer::solveBandedLDLT(
    const std::vector<double>& diag0,
    const std::vector<double>& diag1,
    const std::vector<double>& diag2,
    const std::vector<double>& diag3,
    const std::vector<double>& rhs,
    std::vector<double>& outX
) {
    const size_t n = diag0.size();
    if (n == 0 || rhs.size() != n) {
        return false;
    }

    std::vector<double> D(n, 1.0);
    std::vector<double> l1(n, 0.0);
    std::vector<double> l2(n, 0.0);
    std::vector<double> l3(n, 0.0);

    for (size_t j = 0; j < n; ++j) {
        double dj = diag0[j];
        if (j >= 1) dj -= l1[j - 1] * l1[j - 1] * D[j - 1];
        if (j >= 2) dj -= l2[j - 2] * l2[j - 2] * D[j - 2];
        if (j >= 3) dj -= l3[j - 3] * l3[j - 3] * D[j - 3];
        if (!std::isfinite(dj) || dj < 1e-12) {
            dj = 1e-12;
        }
        D[j] = dj;
        const double invD = 1.0 / dj;

        if (j + 1 < n) {
            double s1 = diag1[j];
            if (j >= 1) s1 -= l2[j - 1] * l1[j - 1] * D[j - 1];
            if (j >= 2) s1 -= l3[j - 2] * l2[j - 2] * D[j - 2];
            l1[j] = s1 * invD;
        }
        if (j + 2 < n) {
            double s2 = diag2[j];
            if (j >= 1) s2 -= l3[j - 1] * l1[j - 1] * D[j - 1];
            l2[j] = s2 * invD;
        }
        if (j + 3 < n) {
            l3[j] = diag3[j] * invD;
        }
    }

    std::vector<double> y(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        double sum = rhs[i];
        if (i >= 1) sum -= l1[i - 1] * y[i - 1];
        if (i >= 2) sum -= l2[i - 2] * y[i - 2];
        if (i >= 3) sum -= l3[i - 3] * y[i - 3];
        y[i] = sum;
    }

    for (size_t i = 0; i < n; ++i) {
        y[i] /= D[i];
    }

    outX.resize(n);
    for (size_t ii = n; ii > 0; --ii) {
        const size_t i = ii - 1;
        double sum = y[i];
        if (i + 1 < n) sum -= l1[i] * outX[i + 1];
        if (i + 2 < n) sum -= l2[i] * outX[i + 2];
        if (i + 3 < n) sum -= l3[i] * outX[i + 3];
        outX[i] = sum;
    }
    return true;
}

ConstrainedPathResult ConstrainedPathOptimizer::solve(
    const ConstrainedPathInput& input,
    const StabilizerConfig& config
) {
    ConstrainedPathResult res;
    if (!input.rawFrameQuats || input.rawFrameQuats->empty()) {
        return res;
    }

    const std::vector<Quat>& rawQuats = *input.rawFrameQuats;
    const size_t n = rawQuats.size();
    const double fps = (input.nominalFps > 1.0) ? input.nominalFps : 30.0;

    const Vec3 defaultGrav(0.0, 1.0, 0.0);
    auto getGrav = [&](size_t i) -> Vec3 {
        if (input.frameGravityW && i < input.frameGravityW->size()) {
            return (*input.frameGravityW)[i];
        }
        return defaultGrav;
    };

    const int horizonQuadDeg = detectGravityQuadrantDeg(getGrav(0));
    res.detectedGravityQuadrantDeg = horizonQuadDeg;

    if (n < 3) {
        res.smoothFrameQuats = rawQuats;
        res.valid = true;
        return res;
    }

    const bool applyHorizon = (config.horizonLockEnabled &&
                               config.horizonMode != HorizonMode::Off &&
                               config.horizonLockAmount > 1e-4);
    const double hAmount = std::min(1.0, std::max(0.0, config.horizonLockAmount));
    const double displayRotRad = static_cast<double>(horizonQuadDeg) * kDeg2Rad;
    const double tiltRad = config.horizonTiltOffsetDeg * kDeg2Rad;
    const Quat screenAlignQ = Quat::fromAxisAngle(Vec3(0.0, 0.0, 1.0), displayRotRad + tiltRad);
    const double kneeRad = 35.0 * kDeg2Rad;
    const double spanRad = 10.0 * kDeg2Rad;

    // 1. Construct base path R_base[i] (horizon leveled if enabled, else raw)
    std::vector<Quat> rBase(n, Quat::identity());
    std::vector<double> rollTwistRad(n, 0.0);
    std::vector<bool> inActionKneePast(n, false);
    const Vec3 zAxis(0.0, 0.0, 1.0);

    for (size_t i = 0; i < n; ++i) {
        const Quat& qRaw = rawQuats[i];
        if (!applyHorizon) {
            rBase[i] = qRaw;
        } else {
            const Quat qHorizon = computeHorizonQuatForForward(qRaw, getGrav(i), screenAlignQ);
            if (config.horizonMode == HorizonMode::Full360Lock) {
                rBase[i] = Quat::slerp(qRaw, qHorizon, hAmount);
                const Quat qCorr = (qRaw.conjugate() * rBase[i]).normalized();
                Quat qSw, qTw;
                Quat::decomposeSwingTwist(qCorr, zAxis, qSw, qTw);
                rollTwistRad[i] = 2.0 * std::atan2(qTw.z, qTw.w);
            } else {
                const Quat qRollDiff = (qRaw.conjugate() * qHorizon).normalized();
                double wrapped = 2.0 * std::atan2(qRollDiff.z, qRollDiff.w);
                while (wrapped > kPi) wrapped -= 2.0 * kPi;
                while (wrapped < -kPi) wrapped += 2.0 * kPi;

                const double absRoll = std::fabs(wrapped);
                double corrRoll = wrapped;
                if (absRoll > kneeRad) {
                    inActionKneePast[i] = true;
                    const double sign = (wrapped >= 0.0) ? 1.0 : -1.0;
                    corrRoll = sign * (kneeRad + spanRad * std::tanh((absRoll - kneeRad) / spanRad));
                }
                rollTwistRad[i] = corrRoll * hAmount;
                rBase[i] = (qRaw * Quat::fromAxisAngle(zAxis, corrRoll * hAmount)).normalized();
            }
        }
    }

    // Known offset c_i = log(R_raw,i^-1 * R_base,i) and base angular velocity omega_base,i
    std::vector<Vec3> cBase(n, Vec3(0.0, 0.0, 0.0));
    for (size_t i = 0; i < n; ++i) {
        cBase[i] = quatLogVec((rawQuats[i].conjugate() * rBase[i]).normalized());
    }

    std::vector<Vec3> omegaBase(n - 1, Vec3(0.0, 0.0, 0.0));
    std::vector<double> omegaBaseX(n - 1, 0.0), omegaBaseY(n - 1, 0.0), omegaBaseZ(n - 1, 0.0);
    for (size_t i = 0; i + 1 < n; ++i) {
        const Vec3 w = quatLogVec((rBase[i].conjugate() * rBase[i + 1]).normalized()) * fps;
        omegaBase[i] = w;
        omegaBaseX[i] = w.x;
        omegaBaseY[i] = w.y;
        omegaBaseZ[i] = w.z;
    }

    // 2. Compute Anti-Drop Pan per-axis weights + smooth-input jitter residual detector
    std::vector<double> panX, panY, panZ;
    computePerAxisPanWeights(omegaBase, fps, panX, panY, panZ, res.avgPanRatio);

    double smoothInputScale = 1.0;
    if (omegaBase.size() >= 4) {
        std::vector<double> d2Omega;
        d2Omega.reserve(omegaBase.size() - 2);
        for (size_t i = 1; i + 1 < omegaBase.size(); ++i) {
            const Vec3 d2 = omegaBase[i + 1] - omegaBase[i] * 2.0 + omegaBase[i - 1];
            d2Omega.push_back(d2.norm());
        }
        std::sort(d2Omega.begin(), d2Omega.end());
        const double q75D2 = d2Omega[(3 * d2Omega.size()) / 4];
        const double hfRamp = std::min(1.0, std::max(0.0, (q75D2 - 0.008) / (0.045 - 0.008)));
        smoothInputScale = hfRamp * hfRamp;
    }

    // 3. Per-axis smoothness mapping (matching Portrait 90/270 vs Landscape 0/180 semantics)
    const double pSmooth = (config.pitchSmoothness >= 0.0) ? config.pitchSmoothness : config.smoothness;
    const double ySmooth = (config.yawSmoothness >= 0.0)   ? config.yawSmoothness   : config.smoothness;
    const double rSmooth = (config.rollSmoothness >= 0.0)  ? config.rollSmoothness  : config.smoothness;
    const bool isPortrait = (config.videoRotationDegrees == 90 || config.videoRotationDegrees == 270);
    const double smoothCamX = isPortrait ? ySmooth : pSmooth;
    const double smoothCamY = isPortrait ? pSmooth : ySmooth;
    const double smoothCamZ = rSmooth;

    // 4. Anisotropic 16:9 crop box bounds in camera tangent space
    const uint32_t width = (input.width >= 320) ? input.width : 1920;
    const uint32_t height = (input.height >= 240) ? input.height : 1080;
    const double fx = (input.fx > 50.0) ? input.fx : 1815.0;
    const double fy = (input.fy > 50.0) ? input.fy : 1815.0;
    const double cx = (input.cx > 10.0) ? input.cx : (0.5 * width);
    const double cy = (input.cy > 10.0) ? input.cy : (0.5 * height);

    const bool hasTightZoomLimit = (config.maxZoomLimit > 1.001 && config.maxZoomLimit <= 1.85);
    const double targetZ = hasTightZoomLimit ? config.maxZoomLimit : 1.48;
    const double halfW = (0.5 * static_cast<double>(width)) / targetZ;
    const double halfH = (0.5 * static_cast<double>(height)) / targetZ;
    const double maxU = (0.5 * static_cast<double>(width)) - 4.1;
    const double maxV = (0.5 * static_cast<double>(height)) - 4.1;
    const double kMaxSwingRad = 24.45 * kDeg2Rad;

    const bool useRS = (config.rollingShutterCorrection &&
                        input.engine != nullptr &&
                        std::fabs(input.readoutTimeSec) > 1e-5 &&
                        input.frameTimestampsSec != nullptr &&
                        input.frameTimestampsSec->size() == n);

    std::vector<double> boxX(n, kMaxSwingRad), boxY(n, kMaxSwingRad), boxZ(n, 0.0);
    double maxSwingReport = hasTightZoomLimit ? 0.0 : 24.45;

    for (size_t i = 0; i < n; ++i) {
        const double cosR = std::fabs(std::cos(rollTwistRad[i]));
        const double sinR = std::fabs(std::sin(rollTwistRad[i]));
        const double maxUr = halfW * cosR + halfH * sinR;
        const double maxVr = halfW * sinR + halfH * cosR;

        double rawLimX = kMaxSwingRad;
        double rawLimY = kMaxSwingRad;
        if (hasTightZoomLimit) {
            const double availU = std::max(2.0, maxU - maxUr);
            const double availV = std::max(2.0, maxV - maxVr);
            rawLimY = std::min(kMaxSwingRad, 0.99 * std::atan(availU / fx));
            rawLimX = std::min(kMaxSwingRad, 0.99 * std::atan(availV / fy));
        } else {
            // Wide zoom mode (>1.85x or unlimited): preserve full 24.45 deg swing envelope
            rawLimY = kMaxSwingRad;
            rawLimX = kMaxSwingRad;
        }

        // Rotate raw sensor bounds into R_base frame (swaps 1080<->1920 cleanly at +-90 deg roll)
        const double projScale = hasTightZoomLimit ? (1.0 / std::max(1.0, (cosR + sinR) * 0.92)) : 1.0;
        boxX[i] = std::min(kMaxSwingRad, (cosR * rawLimX + sinR * rawLimY) * projScale);
        boxY[i] = std::min(kMaxSwingRad, (sinR * rawLimX + cosR * rawLimY) * projScale);
        if (hasTightZoomLimit) {
            maxSwingReport = std::max(maxSwingReport, std::max(boxX[i], boxY[i]) * kRad2Deg);
        }

        if (applyHorizon) {
            boxZ[i] = inActionKneePast[i] ? (2.5 * kDeg2Rad) : 0.0015;
        } else {
            boxZ[i] = hasTightZoomLimit
                ? std::min(15.0 * kDeg2Rad, 0.45 * std::min(rawLimX, rawLimY) + 2.0 * kDeg2Rad)
                : (18.0 * kDeg2Rad);
        }
    }

    // Smooth |sin(rollTwistRad)| V-cusps strictly from below via two-pass min-envelope IIR
    if (hasTightZoomLimit) {
        const double alphaBox = std::exp(-(1.0 / fps) / 0.10);
        for (size_t i = 1; i < n; ++i) {
            boxX[i] = std::min(boxX[i], alphaBox * boxX[i - 1] + (1.0 - alphaBox) * boxX[i]);
            boxY[i] = std::min(boxY[i], alphaBox * boxY[i - 1] + (1.0 - alphaBox) * boxY[i]);
        }
        for (size_t i = n - 1; i > 0; --i) {
            const size_t idx = i - 1;
            boxX[idx] = std::min(boxX[idx], alphaBox * boxX[idx + 1] + (1.0 - alphaBox) * boxX[idx]);
            boxY[idx] = std::min(boxY[idx], alphaBox * boxY[idx + 1] + (1.0 - alphaBox) * boxY[idx]);
        }
    }

    std::vector<double> lx(n), ux(n), ly(n), uy(n), lz(n), uz(n);
    auto updateIntervalsFromBoxes = [&]() {
        for (size_t i = 0; i < n; ++i) {
            // Note: when HorizonLock is on, cBase.x == cBase.y == 0 and twist is locked separately via uZ
            const double cxOff = applyHorizon ? 0.0 : cBase[i].x;
            const double cyOff = applyHorizon ? 0.0 : cBase[i].y;
            const double czOff = applyHorizon ? 0.0 : cBase[i].z;

            lx[i] = std::max(-kMaxSwingRad, -boxX[i] - cxOff);
            ux[i] = std::min(kMaxSwingRad,   boxX[i] - cxOff);
            if (lx[i] > ux[i]) {
                const double m = 0.5 * (lx[i] + ux[i]);
                lx[i] = ux[i] = m;
            }

            ly[i] = std::max(-kMaxSwingRad, -boxY[i] - cyOff);
            uy[i] = std::min(kMaxSwingRad,   boxY[i] - cyOff);
            if (ly[i] > uy[i]) {
                const double m = 0.5 * (ly[i] + uy[i]);
                ly[i] = uy[i] = m;
            }

            lz[i] = -boxZ[i] - czOff;
            uz[i] =  boxZ[i] - czOff;
            if (lz[i] > uz[i]) {
                const double m = 0.5 * (lz[i] + uz[i]);
                lz[i] = uz[i] = m;
            }
        }
    };
    updateIntervalsFromBoxes();

    std::vector<double> uX(n, 0.0), uY(n, 0.0), uZ(n, 0.0);

    const double zoomEconWeight = (config.dynamicZoomEnabled && hasTightZoomLimit)
        ? (2.80 / std::max(0.25, config.maxZoomLimit - 1.0))
        : 0.0;
    AxisParams pxParams{smoothCamX, config.velocityDamping, fps, config.motoActionMode && !isPortrait, zoomEconWeight, smoothInputScale};
    AxisParams pyParams{smoothCamY, config.velocityDamping, fps, config.motoActionMode && isPortrait, zoomEconWeight, smoothInputScale};
    AxisParams pzParams{smoothCamZ, config.velocityDamping, fps, false, zoomEconWeight, smoothInputScale};

    auto applyHorizonProjection = [&](size_t i, const Quat& qIn) -> Quat {
        if (!applyHorizon) {
            return qIn;
        }
        const Quat qHorizonCand = computeHorizonQuatForForward(qIn, getGrav(i), screenAlignQ);
        if (config.horizonMode == HorizonMode::Full360Lock) {
            return (hAmount >= 0.9999) ? qHorizonCand : Quat::slerp(qIn, qHorizonCand, hAmount);
        }
        const Quat qDiff = (qIn.conjugate() * qHorizonCand).normalized();
        double wrapped = 2.0 * std::atan2(qDiff.z, qDiff.w);
        while (wrapped > kPi) wrapped -= 2.0 * kPi;
        while (wrapped < -kPi) wrapped += 2.0 * kPi;
        const double absRoll = std::fabs(wrapped);
        double corrRoll = wrapped;
        if (absRoll > kneeRad) {
            const double sign = (wrapped >= 0.0) ? 1.0 : -1.0;
            corrRoll = sign * (kneeRad + spanRad * std::tanh((absRoll - kneeRad) / spanRad));
        }
        return (qIn * Quat::fromAxisAngle(zAxis, corrRoll * hAmount)).normalized();
    };

    auto reconstructSingleQuat = [&](size_t i, double uxVal, double uyVal, double uzVal) -> Quat {
        const Vec3 uVec(uxVal, uyVal, applyHorizon ? 0.0 : uzVal);
        const Quat qCand = (rBase[i] * quatExpVec(uVec)).normalized();
        return applyHorizonProjection(i, qCand);
    };

    auto reconstructQuats = [&](std::vector<Quat>& outQuats) {
        outQuats.resize(n);
        for (size_t i = 0; i < n; ++i) {
            outQuats[i] = reconstructSingleQuat(i, uX[i], uY[i], uZ[i]);
        }
    };

    auto evalMaxSliceZoomForQuat = [&](size_t i, const Quat& qSm) -> double {
        double zReq = FrameWarpCalculator::computeRequiredZoom(
            (rawQuats[i].conjugate() * qSm).normalized(), width, height, fx, fy, cx, cy, 4.0
        );
        if (useRS) {
            const double tMid = (*input.frameTimestampsSec)[i];
            const Quat qTop = input.engine->sampleRawQuaternionAt(tMid - 0.5 * input.readoutTimeSec);
            const Quat qBot = input.engine->sampleRawQuaternionAt(tMid + 0.5 * input.readoutTimeSec);
            const double zTop = FrameWarpCalculator::computeRequiredZoom(
                (qTop.conjugate() * qSm).normalized(), width, height, fx, fy, cx, cy, 4.0
            );
            const double zBot = FrameWarpCalculator::computeRequiredZoom(
                (qBot.conjugate() * qSm).normalized(), width, height, fx, fy, cx, cy, 4.0
            );
            zReq = std::max(zReq, std::max(zTop, zBot));
        }
        return zReq;
    };

    const int maxOuter = hasTightZoomLimit ? 4 : 1; // Initial solve + up to 3 outer RS refinements
    std::vector<Quat> smoothQuats(n, Quat::identity());

    for (int outer = 0; outer < maxOuter; ++outer) {
        res.outerIterationsUsed = outer + 1;

        solveAxis1DIRLS(omegaBaseX, lx, ux, panX, pxParams, uX);
        solveAxis1DIRLS(omegaBaseY, ly, uy, panY, pyParams, uY);
        if (!applyHorizon) {
            solveAxis1DIRLS(omegaBaseZ, lz, uz, panZ, pzParams, uZ);
        }

        reconstructQuats(smoothQuats);

        if (!hasTightZoomLimit || outer + 1 >= maxOuter) {
            break;
        }

        // Independent per-axis exact RS-slice crop verification via FrameWarpCalculator::computeRequiredZoom.
        // Tighten only the violating axis/side with smooth two-pass IIR bounds without cross-notching pans.
        std::vector<double> capUx = ux, capLx = lx, capUy = uy, capLy = ly;
        bool anyViolation = false;
        const double checkZoomTarget = std::max(1.001, config.maxZoomLimit - 0.003);

        for (size_t i = 0; i < n; ++i) {
            const Quat qCorrMid = (rawQuats[i].conjugate() * smoothQuats[i]).normalized();
            Quat qSw, qTw;
            Quat::decomposeSwingTwist(qCorrMid, zAxis, qSw, qTw);

            const double zTwistOnly = applyHorizon
                ? FrameWarpCalculator::computeRequiredZoom(qTw, width, height, fx, fy, cx, cy, 4.0)
                : 1.0;

            const double zReq = evalMaxSliceZoomForQuat(i, smoothQuats[i]);
            const double effCap = std::max(checkZoomTarget, zTwistOnly + 0.002);
            if (zReq > effCap) {
                anyViolation = true;
                const double availExcess = std::max(1e-4, effCap - 1.0);
                // Find which RS slice (mid/top/bot) maximizes zoom so we pull away from that exact slice's angle
                Vec3 dWorst(0.0, 0.0, 0.0);
                if (useRS) {
                    const double tMid = (*input.frameTimestampsSec)[i];
                    const Quat qTop = input.engine->sampleRawQuaternionAt(tMid - 0.5 * input.readoutTimeSec);
                    const Quat qBot = input.engine->sampleRawQuaternionAt(tMid + 0.5 * input.readoutTimeSec);
                    const double zMidV = FrameWarpCalculator::computeRequiredZoom(qCorrMid, width, height, fx, fy, cx, cy, 4.0);
                    const double zTopV = FrameWarpCalculator::computeRequiredZoom((qTop.conjugate() * smoothQuats[i]).normalized(), width, height, fx, fy, cx, cy, 4.0);
                    const double zBotV = FrameWarpCalculator::computeRequiredZoom((qBot.conjugate() * smoothQuats[i]).normalized(), width, height, fx, fy, cx, cy, 4.0);
                    if (zTopV >= zMidV && zTopV >= zBotV) {
                        dWorst = quatLogVec((qTop.conjugate() * rawQuats[i]).normalized());
                    } else if (zBotV > zMidV && zBotV > zTopV) {
                        dWorst = quatLogVec((qBot.conjugate() * rawQuats[i]).normalized());
                    }
                }

                const Quat qOnlyX = reconstructSingleQuat(i, uX[i], 0.0, uZ[i]);
                const Quat qOnlyY = reconstructSingleQuat(i, 0.0, uY[i], uZ[i]);
                const double zOnlyX = evalMaxSliceZoomForQuat(i, qOnlyX);
                const double zOnlyY = evalMaxSliceZoomForQuat(i, qOnlyY);

                const double cxOff = applyHorizon ? 0.0 : cBase[i].x;
                const double cyOff = applyHorizon ? 0.0 : cBase[i].y;
                const double angX = cxOff + uX[i];
                const double angY = cyOff + uY[i];
                const double phiX = dWorst.x + angX;
                const double phiY = dWorst.y + angY;

                bool tightenedAxis = false;
                if (zOnlyX > effCap) {
                    const double exX = std::max(1e-4, zOnlyX - 1.0);
                    const double rX = std::min(0.92, std::max(0.35, (availExcess / exX) * 0.90));
                    if (phiX >= 0.0) capUx[i] = std::min(capUx[i], angX - (1.0 - rX) * phiX - cxOff);
                    else             capLx[i] = std::max(capLx[i], angX - (1.0 - rX) * phiX - cxOff);
                    tightenedAxis = true;
                }
                if (zOnlyY > effCap) {
                    const double exY = std::max(1e-4, zOnlyY - 1.0);
                    const double rY = std::min(0.92, std::max(0.35, (availExcess / exY) * 0.90));
                    if (phiY >= 0.0) capUy[i] = std::min(capUy[i], angY - (1.0 - rY) * phiY - cyOff);
                    else             capLy[i] = std::max(capLy[i], angY - (1.0 - rY) * phiY - cyOff);
                    tightenedAxis = true;
                }
                if (!tightenedAxis) {
                    // Single-axis diagonal tightening: protect clean panning axis from pitch RS notches
                    const double actualExcess = std::max(1e-4, zReq - 1.0);
                    const bool yProtectedPan = (panY[i] > 0.35 && panX[i] < 0.20 && zOnlyY < effCap - 0.03);
                    const bool xProtectedPan = (panX[i] > 0.35 && panY[i] < 0.20 && zOnlyX < effCap - 0.03);
                    const bool shrinkX = yProtectedPan || (!xProtectedPan && (zOnlyX >= zOnlyY));
                    const double rDiag = std::min(0.93, std::max(0.55, (availExcess / actualExcess) * 0.90));
                    if (shrinkX) {
                        if (phiX >= 0.0) capUx[i] = std::min(capUx[i], angX - (1.0 - rDiag) * phiX - cxOff);
                        else             capLx[i] = std::max(capLx[i], angX - (1.0 - rDiag) * phiX - cxOff);
                    } else {
                        if (phiY >= 0.0) capUy[i] = std::min(capUy[i], angY - (1.0 - rDiag) * phiY - cyOff);
                        else             capLy[i] = std::max(capLy[i], angY - (1.0 - rDiag) * phiY - cyOff);
                    }
                }
            }
        }

        if (!anyViolation) {
            break;
        }

        // Smooth two-pass min/max-envelope IIR on updated [lx, ux] and [ly, uy] intervals (tau = 0.28s)
        const double alphaCap = std::exp(-(1.0 / fps) / 0.28);
        for (size_t i = 1; i < n; ++i) {
            capUx[i] = std::min(capUx[i], alphaCap * capUx[i - 1] + (1.0 - alphaCap) * capUx[i]);
            capUy[i] = std::min(capUy[i], alphaCap * capUy[i - 1] + (1.0 - alphaCap) * capUy[i]);
            capLx[i] = std::max(capLx[i], alphaCap * capLx[i - 1] + (1.0 - alphaCap) * capLx[i]);
            capLy[i] = std::max(capLy[i], alphaCap * capLy[i - 1] + (1.0 - alphaCap) * capLy[i]);
        }
        for (size_t i = n - 1; i > 0; --i) {
            const size_t idx = i - 1;
            capUx[idx] = std::min(capUx[idx], alphaCap * capUx[idx + 1] + (1.0 - alphaCap) * capUx[idx]);
            capUy[idx] = std::min(capUy[idx], alphaCap * capUy[idx + 1] + (1.0 - alphaCap) * capUy[idx]);
            capLx[idx] = std::max(capLx[idx], alphaCap * capLx[idx + 1] + (1.0 - alphaCap) * capLx[idx]);
            capLy[idx] = std::max(capLy[idx], alphaCap * capLy[idx + 1] + (1.0 - alphaCap) * capLy[idx]);
        }
        for (size_t i = 0; i < n; ++i) {
            ux[i] = capUx[i];
            lx[i] = capLx[i];
            if (lx[i] > ux[i]) {
                const double m = 0.5 * (lx[i] + ux[i]);
                lx[i] = ux[i] = m;
            }
            uy[i] = capUy[i];
            ly[i] = capLy[i];
            if (ly[i] > uy[i]) {
                const double m = 0.5 * (ly[i] + uy[i]);
                ly[i] = uy[i] = m;
            }
        }
    }

    // Boundary-saturation harmonic polish for extreme shock clips (leaves interior & pan frames untouched)
    if (n >= 7 && hasTightZoomLimit) {
        std::vector<double> sat(n, 0.0);
        bool anySat = false;
        for (size_t i = 0; i < n; ++i) {
            const double sx = std::fabs(uX[i]) / std::max(boxX[i], 1e-4);
            const double sy = std::fabs(uY[i]) / std::max(boxY[i], 1e-4);
            const double panSuppress = 1.0 - 0.85 * std::max(panX[i], panY[i]);
            sat[i] = std::max(sx, sy) * panSuppress;
            if (sat[i] > 0.80) {
                anySat = true;
            }
        }
        if (anySat) {
            std::vector<double> satSmooth = sat;
            smoothScalarGaussianTwoPass(satSmooth, 3.0);

            constexpr int kRad = 5;
            constexpr double sigma = 2.8;
            double gw[2 * kRad + 1];
            double wNorm = 0.0;
            for (int k = -kRad; k <= kRad; ++k) {
                gw[k + kRad] = std::exp(-static_cast<double>(k * k) / (2.0 * sigma * sigma));
                wNorm += gw[k + kRad];
            }
            for (int k = 0; k < 2 * kRad + 1; ++k) {
                gw[k] /= wNorm;
            }

            for (int pass = 0; pass < 2; ++pass) {
                std::vector<Quat> nextQuats = smoothQuats;
                for (size_t i = 0; i < n; ++i) {
                    const double tSat = std::min(1.0, std::max(0.0, (satSmooth[i] - 0.65) / (0.90 - 0.65)));
                    const double wBlend = tSat * tSat * (3.0 - 2.0 * tSat);
                    if (wBlend <= 1e-4) {
                        continue;
                    }
                    const Quat& qCenter = smoothQuats[i];
                    Vec3 rAvg(0.0, 0.0, 0.0);
                    double localW = 0.0;
                    for (int k = -kRad; k <= kRad; ++k) {
                        const int idx = static_cast<int>(i) + k;
                        if (idx >= 0 && idx < static_cast<int>(n)) {
                            const double w = gw[k + kRad];
                            rAvg = rAvg + quatLogVec(qCenter.conjugate() * smoothQuats[static_cast<size_t>(idx)]) * w;
                            localW += w;
                        }
                    }
                    if (localW > 1e-6) {
                        rAvg = rAvg * (1.0 / localW);
                    }
                    const Quat qAvg = (qCenter * quatExpVec(rAvg)).normalized();
                    const Quat qBlended = Quat::slerp(qCenter, qAvg, wBlend);
                    nextQuats[i] = applyHorizonProjection(i, qBlended);
                }
                smoothQuats = std::move(nextQuats);
            }
        }
    }

    // Smooth decoupled two-pass IIR RS safeguard if any residual slice slightly exceeds maxZoomLimit
    if (hasTightZoomLimit) {
        bool anyResidual = false;
        std::vector<double> pullX(n, 1.0);
        std::vector<double> pullY(n, 1.0);
        std::vector<Vec3> corrSwing(n, Vec3(0.0, 0.0, 0.0));
        std::vector<double> corrTwistZ(n, 0.0);

        auto reconstructPulled = [&](size_t i, double pX, double pY) -> Quat {
            const double pZ = applyHorizon ? 1.0 : std::min(pX, pY);
            const Quat qSwing = quatExpVec(Vec3(pX * corrSwing[i].x, pY * corrSwing[i].y, 0.0));
            const Quat qTwist = quatExpVec(Vec3(0.0, 0.0, pZ * corrTwistZ[i]));
            const Quat qCand = (rawQuats[i] * (qSwing * qTwist)).normalized();
            return applyHorizonProjection(i, qCand);
        };

        for (size_t i = 0; i < n; ++i) {
            const Quat qCorrMid = (rawQuats[i].conjugate() * smoothQuats[i]).normalized();
            Quat qSw, qTw;
            Quat::decomposeSwingTwist(qCorrMid, zAxis, qSw, qTw);
            corrSwing[i] = quatLogVec(qSw);
            corrTwistZ[i] = quatLogVec(qTw).z;

            const double zTwistOnly = applyHorizon
                ? FrameWarpCalculator::computeRequiredZoom(qTw, width, height, fx, fy, cx, cy, 4.0)
                : 1.0;
            if (zTwistOnly <= config.maxZoomLimit && evalMaxSliceZoomForQuat(i, smoothQuats[i]) > config.maxZoomLimit) {
                anyResidual = true;
                const double zZeroXKeepY = evalMaxSliceZoomForQuat(i, reconstructPulled(i, 0.0, 1.0));
                const double zKeepXZeroY = evalMaxSliceZoomForQuat(i, reconstructPulled(i, 1.0, 0.0));

                auto bisectX = [&](double pYFixed) -> double {
                    double lo = 0.0, hi = 1.0;
                    for (int step = 0; step < 10; ++step) {
                        const double mid = 0.5 * (lo + hi);
                        if (evalMaxSliceZoomForQuat(i, reconstructPulled(i, mid, pYFixed)) <= config.maxZoomLimit) {
                            lo = mid;
                        } else {
                            hi = mid;
                        }
                    }
                    return lo;
                };
                auto bisectY = [&](double pXFixed) -> double {
                    double lo = 0.0, hi = 1.0;
                    for (int step = 0; step < 10; ++step) {
                        const double mid = 0.5 * (lo + hi);
                        if (evalMaxSliceZoomForQuat(i, reconstructPulled(i, pXFixed, mid)) <= config.maxZoomLimit) {
                            lo = mid;
                        } else {
                            hi = mid;
                        }
                    }
                    return lo;
                };

                if (zZeroXKeepY <= config.maxZoomLimit && zKeepXZeroY > config.maxZoomLimit) {
                    pullX[i] = bisectX(1.0);
                    pullY[i] = 1.0;
                } else if (zKeepXZeroY <= config.maxZoomLimit && zZeroXKeepY > config.maxZoomLimit) {
                    pullX[i] = 1.0;
                    pullY[i] = bisectY(1.0);
                } else if (zZeroXKeepY <= config.maxZoomLimit && zKeepXZeroY <= config.maxZoomLimit) {
                    if (zKeepXZeroY >= zZeroXKeepY) {
                        pullX[i] = bisectX(1.0);
                        pullY[i] = 1.0;
                    } else {
                        pullX[i] = 1.0;
                        pullY[i] = bisectY(1.0);
                    }
                } else {
                    double pX = bisectX(0.0);
                    double pY = bisectY(0.0);
                    if (evalMaxSliceZoomForQuat(i, reconstructPulled(i, pX, pY)) > config.maxZoomLimit) {
                        double lo = 0.0, hi = 1.0;
                        for (int step = 0; step < 10; ++step) {
                            const double mid = 0.5 * (lo + hi);
                            if (evalMaxSliceZoomForQuat(i, reconstructPulled(i, mid * pX, mid * pY)) <= config.maxZoomLimit) {
                                lo = mid;
                            } else {
                                hi = mid;
                            }
                        }
                        pX *= lo;
                        pY *= lo;
                    }
                    pullX[i] = pX;
                    pullY[i] = pY;
                }
            }
        }
        if (anyResidual) {
            const double dtSafeguard = 1.0 / fps;
            const double alphaPull = std::exp(-dtSafeguard / 0.32);
            for (size_t i = 1; i < n; ++i) {
                pullX[i] = std::min(pullX[i], alphaPull * pullX[i - 1] + (1.0 - alphaPull) * pullX[i]);
                pullY[i] = std::min(pullY[i], alphaPull * pullY[i - 1] + (1.0 - alphaPull) * pullY[i]);
            }
            for (size_t i = n - 1; i > 0; --i) {
                const size_t idx = i - 1;
                pullX[idx] = std::min(pullX[idx], alphaPull * pullX[idx + 1] + (1.0 - alphaPull) * pullX[idx]);
                pullY[idx] = std::min(pullY[idx], alphaPull * pullY[idx + 1] + (1.0 - alphaPull) * pullY[idx]);
            }
            for (size_t i = 0; i < n; ++i) {
                if (pullX[i] < 0.9999 || pullY[i] < 0.9999) {
                    smoothQuats[i] = reconstructPulled(i, pullX[i], pullY[i]);
                }
            }
        }
    }

    // Final exact hard <= 24.45 deg swing safeguard & horizon lock check
    for (size_t i = 0; i < n; ++i) {
        const Quat qCorr = (rawQuats[i].conjugate() * smoothQuats[i]).normalized();
        Quat qSwing, qTwist;
        Quat::decomposeSwingTwist(qCorr, zAxis, qSwing, qTwist);
        const Vec3 rSwing = quatLogVec(qSwing);
        const double mag = std::hypot(rSwing.x, rSwing.y);
        if (mag > kMaxSwingRad) {
            const double sc = kMaxSwingRad / mag;
            const Quat qClamped = (quatExpVec(Vec3(rSwing.x * sc, rSwing.y * sc, 0.0)) * qTwist).normalized();
            smoothQuats[i] = applyHorizonProjection(i, (rawQuats[i] * qClamped).normalized());
        }
    }

    res.smoothFrameQuats = std::move(smoothQuats);
    res.maxSwingCapDeg = maxSwingReport;
    res.valid = true;
    return res;
}

} // namespace gf
