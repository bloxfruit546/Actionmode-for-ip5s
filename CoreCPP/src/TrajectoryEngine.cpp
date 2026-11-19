/**
 * Unified World <-> Camera <-> Pixel Convention:
 *   - q_raw(t) = q_WC(t): Camera-to-World orientation (P_world = R_WC * P_cam).
 *   - P_cam(t) = R_CW(t) * P_world, where R_CW(t) = R_WC(t)^T = q_raw(t).conjugate().
 *   - Inverse Homography in FrameWarpCalculator: H_i(r) = K_real * R_CW(t_r) * R_CW_smooth(i)^T * K_virt^(-1).
 */
#include "TrajectoryEngine.hpp"

#include <algorithm>
#include <cmath>

namespace gf {

namespace {

constexpr double kPi = 3.14159265358979323846;

Vec3 applyAxisMatrix(const double m[9], const Vec3& v) {
    return Vec3(
        m[0] * v.x + m[1] * v.y + m[2] * v.z,
        m[3] * v.x + m[4] * v.y + m[5] * v.z,
        m[6] * v.x + m[7] * v.y + m[8] * v.z
    );
}

Quat slerpShortest(const Quat& a, const Quat& b, double t) {
    return Quat::slerp(a, b, t);
}

Vec3 quatLog(const Quat& qIn) {
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

Quat quatExp(const Vec3& r) {
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

double rotationAngleBetween(const Quat& a, const Quat& b) {
    Quat dq = (a.conjugate() * b).normalized();
    if (dq.w < 0.0) {
        dq = Quat(-dq.w, -dq.x, -dq.y, -dq.z);
    }
    const double clampedW = std::min(1.0, std::max(-1.0, dq.w));
    return 2.0 * std::acos(clampedW);
}

double computeAngularAccelRms(const std::vector<Quat>& quats, double dt) {
    if (quats.size() < 3 || dt <= 1e-6) {
        return 0.0;
    }
    double sumSq = 0.0;
    size_t count = 0;
    for (size_t i = 1; i + 1 < quats.size(); ++i) {
        const Quat dq1 = (quats[i - 1].conjugate() * quats[i]).normalized();
        const Quat dq2 = (quats[i].conjugate() * quats[i + 1]).normalized();
        const double diffAngle = rotationAngleBetween(dq1, dq2);
        const double accel = diffAngle / (dt * dt);
        sumSq += accel * accel;
        ++count;
    }
    return (count > 0) ? std::sqrt(sumSq / static_cast<double>(count)) : 0.0;
}

double computeAlphaForAxis(double smoothness, double panActivity, double velocityDamping, double dtF) {
    const double s = std::min(1.0, std::max(0.0, smoothness));
    if (s <= 1e-4) {
        return 0.0;
    }
    // Cinema-grade tauBase: ~1.28s at s=0.65, ~2.80s at s=0.85, ~4.16s at s=0.96, 4.50s at s=1.0
    const double heavyBoost = (s > 0.75) ? (0.50 * std::pow((s - 0.75) / 0.25, 1.3)) : 0.0;
    const double tauBase = 0.02 + 0.90 * s + 3.08 * std::pow(s, 3.5) + heavyBoost;
    // Only reduce tau during genuine intentional panning (panActivity > 0)
    const double tauEff = tauBase / (1.0 + velocityDamping * panActivity * 4.0);
    return std::exp(-dtF / std::max(tauEff, 1e-4));
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

double computeRollErrorDeg(const Quat& qWC, const Vec3& gWorld, int quadrantDegrees) {
    const double rotRad = static_cast<double>(quadrantDegrees) * (kPi / 180.0);
    const Quat rotQ = Quat::fromAxisAngle(Vec3(0.0, 0.0, 1.0), -rotRad);
    const Vec3 screenRightCam = rotQ.rotateVector(Vec3(1.0, 0.0, 0.0));
    const Vec3 screenDownCam = rotQ.rotateVector(Vec3(0.0, 1.0, 0.0));

    const Vec3 rightW = qWC.rotateVector(screenRightCam).normalized();
    const Vec3 downW = qWC.rotateVector(screenDownCam).normalized();
    const Vec3 forwardW = qWC.rotateVector(Vec3(0.0, 0.0, 1.0)).normalized();

    Vec3 gProj = gWorld - forwardW * gWorld.dot(forwardW);
    if (gProj.normSquared() < 1e-6) {
        return 0.0;
    }
    gProj = gProj.normalized();

    const double sinRoll = gProj.dot(rightW);
    const double cosRoll = gProj.dot(downW);
    return std::atan2(sinRoll, cosRoll) * (180.0 / kPi);
}

int detectGravityQuadrantDeg(const Vec3& gWorld) {
    if (std::fabs(gWorld.y) >= std::fabs(gWorld.x)) {
        return (gWorld.y >= 0.0) ? 0 : 180;
    }
    return (gWorld.x >= 0.0) ? 90 : 270;
}

// 2nd-order Butterworth LPF (fc = 24.0 Hz) applied forward-backward (zero phase) to suppress MEMS white noise
void zeroPhaseButterworthLPF(std::vector<Vec3>& data, double fs, double fc) {
    if (data.size() < 12 || fs <= 2.2 * fc) {
        return;
    }
    const double w0 = 2.0 * kPi * fc / fs;
    const double cosW0 = std::cos(w0);
    const double sinW0 = std::sin(w0);
    const double alpha = sinW0 / std::sqrt(2.0); // Q = 1/sqrt(2)

    const double a0 = 1.0 + alpha;
    const double b0 = ((1.0 - cosW0) * 0.5) / a0;
    const double b1 = (1.0 - cosW0) / a0;
    const double b2 = b0;
    const double a1 = (-2.0 * cosW0) / a0;
    const double a2 = (1.0 - alpha) / a0;

    auto runPass = [&](bool reverse) {
        const size_t n = data.size();
        auto getAt = [&](size_t idx) -> Vec3 {
            return reverse ? data[n - 1 - idx] : data[idx];
        };
        auto setAt = [&](size_t idx, const Vec3& v) {
            if (reverse) {
                data[n - 1 - idx] = v;
            } else {
                data[idx] = v;
            }
        };

        Vec3 x1 = getAt(0), x2 = getAt(0);
        Vec3 y1 = getAt(0), y2 = getAt(0);
        for (size_t i = 0; i < n; ++i) {
            const Vec3 x0 = getAt(i);
            const Vec3 y0(
                b0 * x0.x + b1 * x1.x + b2 * x2.x - a1 * y1.x - a2 * y2.x,
                b0 * x0.y + b1 * x1.y + b2 * x2.y - a1 * y1.y - a2 * y2.y,
                b0 * x0.z + b1 * x1.z + b2 * x2.z - a1 * y1.z - a2 * y2.z
            );
            setAt(i, y0);
            x2 = x1; x1 = x0;
            y2 = y1; y1 = y0;
        }
    };

    runPass(false);
    runPass(true);
}

Vec3 catmullRomVec3(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, double t) {
    const double t2 = t * t;
    const double t3 = t2 * t;
    auto eval = [&](double v0, double v1, double v2, double v3) -> double {
        return 0.5 * (
            (2.0 * v1) +
            (-v0 + v2) * t +
            (2.0 * v0 - 5.0 * v1 + 4.0 * v2 - v3) * t2 +
            (-v0 + 3.0 * v1 - 3.0 * v2 + v3) * t3
        );
    };
    return Vec3(eval(p0.x, p1.x, p2.x, p3.x),
                eval(p0.y, p1.y, p2.y, p3.y),
                eval(p0.z, p1.z, p2.z, p3.z));
}

} // namespace

void TrajectoryEngine::setImuData(
    const GFFileHeader& header,
    const std::vector<GFGyroRecord>& gyroRecords,
    const std::vector<GFAccelRecord>& accelRecords,
    const double* customAxisMatrix,
    bool motoActionMode,
    double rollGyroScale
) {
    integratedSamples_.clear();
    nominalFps_ = (header.nominal_fps > 1.0) ? header.nominal_fps : 30.0;

    if (gyroRecords.empty()) {
        return;
    }

    const double defaultMat[9] = {0, -1, 0, -1, 0, 0, 0, 0, -1};
    const double* M = customAxisMatrix ? customAxisMatrix : defaultMat;

    std::vector<GFGyroRecord> sortedGyro = gyroRecords;
    std::sort(sortedGyro.begin(), sortedGyro.end(), [](const GFGyroRecord& a, const GFGyroRecord& b) {
        return a.timestamp_sec < b.timestamp_sec;
    });

    std::vector<GFGyroRecord> dedupGyro;
    dedupGyro.reserve(sortedGyro.size());
    for (const auto& g : sortedGyro) {
        if (dedupGyro.empty() || (g.timestamp_sec - dedupGyro.back().timestamp_sec) > 1e-6) {
            dedupGyro.push_back(g);
        }
    }

    std::vector<GFAccelRecord> sortedAccel = accelRecords;
    std::sort(sortedAccel.begin(), sortedAccel.end(), [](const GFAccelRecord& a, const GFAccelRecord& b) {
        return a.timestamp_sec < b.timestamp_sec;
    });

    const Vec3 biasIos(header.gyro_bias_x, header.gyro_bias_y, header.gyro_bias_z);

    // Pass 1: Zero-Phase 24Hz MEMS White-Noise Pre-Filter (when fs > 55Hz) + 4x Catmull-Rom RK4 integration
    const size_t nGyro = dedupGyro.size();
    std::vector<Quat> qPureGyro(nGyro, Quat::identity());
    std::vector<Vec3> omegaCamArr(nGyro);

    const double clampedRollScale = (rollGyroScale >= 0.92 && rollGyroScale <= 1.08) ? rollGyroScale : 1.0;
    double dtSum = 0.0;
    for (size_t i = 0; i < nGyro; ++i) {
        const auto& g = dedupGyro[i];
        const Vec3 rawOmegaIos(g.gx - biasIos.x, g.gy - biasIos.y, g.gz - biasIos.z);
        Vec3 wCam = applyAxisMatrix(M, rawOmegaIos);
        wCam.z *= clampedRollScale;
        omegaCamArr[i] = wCam;
        if (i > 0) {
            dtSum += std::min(0.1, std::max(1e-6, g.timestamp_sec - dedupGyro[i - 1].timestamp_sec));
        }
    }
    const double avgDt = (nGyro > 1) ? (dtSum / static_cast<double>(nGyro - 1)) : (1.0 / 90.0);
    const double fsEst = 1.0 / std::max(avgDt, 1e-4);
    if (motoActionMode) {
        // Full-bandwidth Nyquist gyro for 20-44Hz road/engine buzz: only filter at 42 Hz if fs > 95 Hz
        if (fsEst > 95.0) {
            zeroPhaseButterworthLPF(omegaCamArr, fsEst, 42.0);
        }
    } else if (fsEst > 55.0) {
        zeroPhaseButterworthLPF(omegaCamArr, fsEst, 24.0);
    }

    const int subSteps = motoActionMode ? 8 : 4;
    const double invSub = 1.0 / static_cast<double>(subSteps);
    for (size_t i = 1; i < nGyro; ++i) {
        const double dt = std::min(0.1, std::max(1e-6, dedupGyro[i].timestamp_sec - dedupGyro[i - 1].timestamp_sec));
        const Vec3& w0 = omegaCamArr[(i >= 2) ? (i - 2) : 0];
        const Vec3& w1 = omegaCamArr[i - 1];
        const Vec3& w2 = omegaCamArr[i];
        const Vec3& w3 = omegaCamArr[(i + 1 < nGyro) ? (i + 1) : (nGyro - 1)];

        Quat qSub = qPureGyro[i - 1];
        const double dtSub = dt * invSub;
        for (int s = 0; s < subSteps; ++s) {
            const double uMid = (static_cast<double>(s) + 0.5) * invSub;
            const Vec3 wMid = catmullRomVec3(w0, w1, w2, w3, uMid);
            qSub = Quat::integrateGyroStep(qSub, wMid, dtSub);
        }
        qPureGyro[i] = qSub;
    }

    // Pass 2: Zero-Phase VQF-Style Offline Gravity Estimator (tau_g = 3.0s)
    // Transforms each accelerometer sample into inertial frame via qPureGyro, weights by |norm-1| & jerk,
    // and runs forward-backward zero-phase weighted LPF so lateral shakes produce ZERO horizon sway.
    initGravityCam_ = Vec3(0.0, 1.0, 0.0);
    std::vector<Vec3> gWorldPerGyro(nGyro, initGravityCam_);

    if (!sortedAccel.empty()) {
        const size_t nAcc = sortedAccel.size();
        std::vector<Vec3> gInertialMeas(nAcc);
        std::vector<double> accWeight(nAcc, 0.001);
        size_t gIdx = 0;

        // Pre-estimate straight-line initial gravity over first 0.5s for cornering yaw-rate detection
        Vec3 gApproxWorld(0.0, 1.0, 0.0);
        {
            const double t0 = sortedAccel.front().timestamp_sec;
            Vec3 sumA(0.0, 0.0, 0.0);
            for (const auto& a : sortedAccel) {
                if (a.timestamp_sec > t0 + 0.5) break;
                sumA = sumA + applyAxisMatrix(M, Vec3(a.ax, a.ay, a.az));
            }
            if (sumA.normSquared() > 1e-6) {
                gApproxWorld = sumA.normalized();
            }
        }

        for (size_t k = 0; k < nAcc; ++k) {
            const auto& a = sortedAccel[k];
            while (gIdx + 1 < nGyro &&
                   std::fabs(dedupGyro[gIdx + 1].timestamp_sec - a.timestamp_sec) <=
                   std::fabs(dedupGyro[gIdx].timestamp_sec - a.timestamp_sec)) {
                ++gIdx;
            }
            const Vec3 aCam = applyAxisMatrix(M, Vec3(a.ax, a.ay, a.az));
            const double aNorm = aCam.norm();

            double jerk = 0.0;
            if (k > 0) {
                const double dtA = std::max(1e-4, a.timestamp_sec - sortedAccel[k - 1].timestamp_sec);
                const Vec3 prevCam = applyAxisMatrix(M, Vec3(sortedAccel[k - 1].ax, sortedAccel[k - 1].ay, sortedAccel[k - 1].az));
                jerk = (aCam - prevCam).norm() / dtA;
            }

            const double magDev = std::fabs(aNorm - 1.0);
            const double omegaMag = omegaCamArr[gIdx].norm();
            double w = motoActionMode ? 1e-5 : 1e-4;
            const double maxMagDev = motoActionMode ? 0.05 : 0.08;
            const double maxJerk = motoActionMode ? 2.0 : 4.0;
            bool cornering = false;
            if (motoActionMode) {
                const Vec3 gCamApprox = qPureGyro[gIdx].conjugate().rotateVector(gApproxWorld).normalized();
                const double omegaYaw = std::fabs(omegaCamArr[gIdx].dot(gCamApprox));
                if (omegaYaw > 0.07) {
                    cornering = true;
                }
            }
            if (!cornering && omegaMag < 0.20 && magDev < 0.04 && jerk < 1.0) {
                // Static anchor / held-tilt settle: fast gravity convergence (tau_eff ~ 0.20s)
                w = 15.0;
            } else if (!cornering && omegaMag <= 0.50 && magDev <= maxMagDev && jerk <= maxJerk) {
                const double wMag = (magDev <= 0.03) ? 1.0 : (1.0 - (magDev - 0.03) / (maxMagDev - 0.03));
                const double wJerk = (jerk <= 1.2) ? 1.0 : (1.0 - (jerk - 1.2) / (maxJerk - 1.2));
                w = std::max(motoActionMode ? 1e-5 : 1e-4, wMag * wJerk);
            }
            accWeight[k] = w;
            gInertialMeas[k] = (aNorm > 1e-4) ? qPureGyro[gIdx].rotateVector(aCam * (1.0 / aNorm)) : Vec3(0.0, 1.0, 0.0);
        }

        // Weighted initial estimate
        Vec3 gMean(0.0, 0.0, 0.0);
        double wSum = 0.0;
        for (size_t k = 0; k < nAcc; ++k) {
            gMean = gMean + gInertialMeas[k] * accWeight[k];
            wSum += accWeight[k];
        }
        if (gMean.normSquared() > 1e-6 && wSum > 0.0) {
            initGravityCam_ = gMean.normalized();
        } else {
            // Fallback: unweighted average of first 0.5s
            const double t0 = sortedAccel.front().timestamp_sec;
            Vec3 sumA(0.0, 0.0, 0.0);
            for (const auto& a : sortedAccel) {
                if (a.timestamp_sec > t0 + 0.5) break;
                sumA = sumA + applyAxisMatrix(M, Vec3(a.ax, a.ay, a.az));
            }
            if (sumA.normSquared() > 1e-6) {
                initGravityCam_ = sumA.normalized();
            }
        }

        // Forward-Backward Zero-Phase Weighted LPF (tau_g = 12.0s in MotoMode, 3.0s normal)
        const double tauG = motoActionMode ? 12.0 : 3.0;
        std::vector<Vec3> gFwd(nAcc, initGravityCam_);
        gFwd[0] = (gInertialMeas[0] * accWeight[0] + initGravityCam_ * (1.0 - accWeight[0])).normalized();
        for (size_t k = 1; k < nAcc; ++k) {
            const double dtA = std::min(0.1, std::max(1e-4, sortedAccel[k].timestamp_sec - sortedAccel[k - 1].timestamp_sec));
            const double effAlpha = std::exp(-dtA * accWeight[k] / tauG);
            gFwd[k] = (gFwd[k - 1] * effAlpha + gInertialMeas[k] * (1.0 - effAlpha)).normalized();
        }

        std::vector<Vec3> gSmooth = gFwd;
        for (size_t k = nAcc - 1; k > 0; --k) {
            const size_t idx = k - 1;
            const double dtA = std::min(0.1, std::max(1e-4, sortedAccel[k].timestamp_sec - sortedAccel[idx].timestamp_sec));
            const double effAlpha = std::exp(-dtA * accWeight[idx] / tauG);
            gSmooth[idx] = (gSmooth[idx + 1] * effAlpha + gFwd[idx] * (1.0 - effAlpha)).normalized();
        }

        initGravityCam_ = gSmooth[0];

        // Map smoothed gravity back to gyro timeline + apply gentle drift correction using clean samples
        size_t aIdx = 0;
        integratedSamples_.reserve(nGyro);
        Quat q = Quat::identity();
        const double kpMahony = motoActionMode ? 0.25 : 0.35;
        const double minMahonyWeight = motoActionMode ? 0.50 : 0.20;

        for (size_t i = 0; i < nGyro; ++i) {
            const double tG = dedupGyro[i].timestamp_sec;
            while (aIdx + 1 < nAcc &&
                   std::fabs(sortedAccel[aIdx + 1].timestamp_sec - tG) <=
                   std::fabs(sortedAccel[aIdx].timestamp_sec - tG)) {
                ++aIdx;
            }
            gWorldPerGyro[i] = gSmooth[aIdx];

            if (i > 0) {
                const double dt = std::min(0.1, std::max(1e-6, tG - dedupGyro[i - 1].timestamp_sec));
                Vec3 feedback(0.0, 0.0, 0.0);
                if (accWeight[aIdx] > minMahonyWeight && std::fabs(sortedAccel[aIdx].timestamp_sec - tG) < 0.05) {
                    const Vec3 aCam = applyAxisMatrix(M, Vec3(sortedAccel[aIdx].ax, sortedAccel[aIdx].ay, sortedAccel[aIdx].az)).normalized();
                    const Vec3 gEst = q.conjugate().rotateVector(gSmooth[aIdx]);
                    const double mahonyW = std::min(2.5, accWeight[aIdx]);
                    feedback = aCam.cross(gEst) * (kpMahony * mahonyW);
                }
                const Vec3& w0 = omegaCamArr[(i >= 2) ? (i - 2) : 0];
                const Vec3& w1 = omegaCamArr[i - 1];
                const Vec3& w2 = omegaCamArr[i];
                const Vec3& w3 = omegaCamArr[(i + 1 < nGyro) ? (i + 1) : (nGyro - 1)];
                const double dtSub = dt * invSub;
                for (int s = 0; s < subSteps; ++s) {
                    const double uMid = (static_cast<double>(s) + 0.5) * invSub;
                    const Vec3 wMid = catmullRomVec3(w0, w1, w2, w3, uMid) + feedback;
                    q = Quat::integrateGyroStep(q, wMid, dtSub);
                }
            }
            integratedSamples_.push_back({tG, q, omegaCamArr[i], gSmooth[aIdx]});
        }
        return;
    }

    integratedSamples_.reserve(nGyro);
    for (size_t i = 0; i < nGyro; ++i) {
        integratedSamples_.push_back({dedupGyro[i].timestamp_sec, qPureGyro[i], omegaCamArr[i], initGravityCam_});
    }
}

Quat TrajectoryEngine::sampleRawQuaternionAt(double timestampSec) const {
    if (integratedSamples_.empty()) {
        return Quat::identity();
    }
    if (integratedSamples_.size() == 1) {
        return integratedSamples_.front().quat;
    }

    if (timestampSec <= integratedSamples_.front().timestampSec) {
        const double dt = std::max(-0.100, timestampSec - integratedSamples_.front().timestampSec);
        return Quat::integrateGyroStep(
            integratedSamples_.front().quat,
            integratedSamples_.front().omegaCam,
            dt
        );
    }
    if (timestampSec >= integratedSamples_.back().timestampSec) {
        const double dt = std::min(0.100, timestampSec - integratedSamples_.back().timestampSec);
        return Quat::integrateGyroStep(
            integratedSamples_.back().quat,
            integratedSamples_.back().omegaCam,
            dt
        );
    }

    auto it = std::lower_bound(
        integratedSamples_.begin(),
        integratedSamples_.end(),
        timestampSec,
        [](const ImuSample& s, double t) { return s.timestampSec < t; }
    );

    if (it == integratedSamples_.begin()) {
        return it->quat;
    }
    const ImuSample& s1 = *it;
    const ImuSample& s0 = *(it - 1);
    const double span = s1.timestampSec - s0.timestampSec;
    if (span <= 1e-9) {
        return s0.quat;
    }
    const double u = (timestampSec - s0.timestampSec) / span;
    // Cubic Hermite C1 spline in so(3) tangent space using exact angular velocity tangents
    const Vec3 p1 = quatLog(s0.quat.conjugate() * s1.quat);
    const Vec3 m0 = s0.omegaCam * span;
    const Vec3 m1 = s1.omegaCam * span;
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double h10 = u3 - 2.0 * u2 + u;
    const double h01 = -2.0 * u3 + 3.0 * u2;
    const double h11 = u3 - u2;
    const Vec3 rInterp = m0 * h10 + p1 * h01 + m1 * h11;
    return (s0.quat * quatExp(rInterp)).normalized();
}

Vec3 TrajectoryEngine::sampleGravityWorldAt(double timestampSec) const {
    if (integratedSamples_.empty()) {
        return initGravityCam_;
    }
    if (timestampSec <= integratedSamples_.front().timestampSec) {
        return integratedSamples_.front().gravityWorld;
    }
    if (timestampSec >= integratedSamples_.back().timestampSec) {
        return integratedSamples_.back().gravityWorld;
    }
    auto it = std::lower_bound(
        integratedSamples_.begin(),
        integratedSamples_.end(),
        timestampSec,
        [](const ImuSample& s, double t) { return s.timestampSec < t; }
    );
    if (it == integratedSamples_.begin()) {
        return it->gravityWorld;
    }
    return it->gravityWorld;
}

TrajectoryResult TrajectoryEngine::solve(
    const std::vector<GFFrameRecord>& frames,
    const StabilizerConfig& config
) const {
    TrajectoryResult result;
    if (frames.empty()) {
        return result;
    }

    const size_t n = frames.size();
    result.rawFrameQuats.resize(n, Quat::identity());
    result.smoothFrameQuats.resize(n, Quat::identity());
    result.frameTimestampsSec.resize(n, 0.0);

    std::vector<Vec3> frameGravityW(n, initGravityCam_);
    const double offsetSec = config.timeOffsetMs * 0.001;
    for (size_t i = 0; i < n; ++i) {
        const double tMid = frames[i].pts_sec
            + 0.5 * static_cast<double>(frames[i].exposure_duration_sec)
            + 0.5 * static_cast<double>(frames[i].rolling_shutter_sec)
            + offsetSec;
        result.frameTimestampsSec[i] = tMid;
        result.rawFrameQuats[i] = sampleRawQuaternionAt(tMid);
        frameGravityW[i] = sampleGravityWorldAt(tMid);
    }

    if (n < 3) {
        result.smoothFrameQuats = result.rawFrameQuats;
        result.valid = true;
        return result;
    }

    const double dtF = 1.0 / nominalFps_;

    // True Net-Displacement Pan Detector over +-0.50s window:
    // panRatio = D_i / (L_i + 1e-4). Only damp tau when panRatio > 0.65 AND D_i > 12 deg.
    const int radius = std::max(2, static_cast<int>(std::round(0.50 * nominalFps_)));
    std::vector<double> stepAngles(n, 0.0);
    for (size_t i = 1; i < n; ++i) {
        stepAngles[i] = rotationAngleBetween(result.rawFrameQuats[i - 1], result.rawFrameQuats[i]);
    }

    std::vector<double> panActivity(n, 0.0);
    std::vector<double> panRatioArr(n, 0.0);
    double panRatioSum = 0.0;
    const double minNetDispRad = 12.0 * (kPi / 180.0);

    for (size_t i = 0; i < n; ++i) {
        const size_t lo = static_cast<size_t>(std::max(0, static_cast<int>(i) - radius));
        const size_t hi = static_cast<size_t>(std::min(static_cast<int>(n) - 1, static_cast<int>(i) + radius));
        double pathLen = 0.0;
        for (size_t k = lo + 1; k <= hi; ++k) {
            pathLen += stepAngles[k];
        }
        const double netDisp = rotationAngleBetween(result.rawFrameQuats[lo], result.rawFrameQuats[hi]);
        const double ratio = netDisp / (pathLen + 1e-4);
        panRatioArr[i] = std::min(1.0, std::max(0.0, ratio));
        panRatioSum += ratio;

        if (ratio > 0.65 && netDisp > minNetDispRad) {
            panActivity[i] = std::min(1.0, (ratio - 0.65) / 0.35);
        }
    }
    result.avgPanRatio = panRatioSum / static_cast<double>(n);

    const double pSmooth = config.pitchSmoothness;
    const double ySmooth = config.yawSmoothness;
    const double rSmooth = config.rollSmoothness;
    const double maxSmooth = std::max(pSmooth, std::max(ySmooth, rSmooth));

    if (pSmooth <= 1e-4 && ySmooth <= 1e-4 && rSmooth <= 1e-4 && !config.horizonLockEnabled) {
        result.smoothFrameQuats = result.rawFrameQuats;
    } else {
        // 3-Pass (or 4-Pass for Heavy Action smoothness > 0.75) Forward-Backward Zero-Phase Smoother
        const int numPasses = (maxSmooth > 0.75) ? 4 : 3;
        std::vector<Quat> current = result.rawFrameQuats;
        std::vector<Quat> qFwd(n);
        std::vector<Quat> qBwd(n);

        for (int pass = 0; pass < numPasses; ++pass) {
            qFwd[0] = current[0];
            for (size_t i = 1; i < n; ++i) {
                const double yawTurnActivity = config.motoActionMode
                    ? std::max(panActivity[i], std::min(1.0, std::max(0.0, (panRatioArr[i] - 0.60) / 0.25)))
                    : panActivity[i];
                const double pitchActivity = config.motoActionMode ? (panActivity[i] * 0.25) : panActivity[i];
                const double aPitch = computeAlphaForAxis(pSmooth, pitchActivity, config.velocityDamping, dtF);
                const double aYaw = computeAlphaForAxis(ySmooth, yawTurnActivity, config.velocityDamping, dtF);
                const double aRoll = computeAlphaForAxis(rSmooth, pitchActivity, config.velocityDamping, dtF);

                const Vec3 rDiff = quatLog(qFwd[i - 1].conjugate() * current[i]);
                const Vec3 rStep(
                    rDiff.x * (1.0 - aPitch),
                    rDiff.y * (1.0 - aYaw),
                    rDiff.z * (1.0 - aRoll)
                );
                qFwd[i] = (qFwd[i - 1] * quatExp(rStep)).normalized();
            }

            qBwd[n - 1] = qFwd[n - 1];
            for (size_t i = n - 1; i > 0; --i) {
                const size_t idx = i - 1;
                const double yawTurnActivity = config.motoActionMode
                    ? std::max(panActivity[idx], std::min(1.0, std::max(0.0, (panRatioArr[idx] - 0.60) / 0.25)))
                    : panActivity[idx];
                const double pitchActivity = config.motoActionMode ? (panActivity[idx] * 0.25) : panActivity[idx];
                const double aPitch = computeAlphaForAxis(pSmooth, pitchActivity, config.velocityDamping, dtF);
                const double aYaw = computeAlphaForAxis(ySmooth, yawTurnActivity, config.velocityDamping, dtF);
                const double aRoll = computeAlphaForAxis(rSmooth, pitchActivity, config.velocityDamping, dtF);

                const Vec3 rDiff = quatLog(qBwd[idx + 1].conjugate() * qFwd[idx]);
                const Vec3 rStep(
                    rDiff.x * (1.0 - aPitch),
                    rDiff.y * (1.0 - aYaw),
                    rDiff.z * (1.0 - aRoll)
                );
                qBwd[idx] = (qBwd[idx + 1] * quatExp(rStep)).normalized();
            }

            current = std::move(qBwd);
            qBwd.resize(n);
        }

        result.smoothFrameQuats = std::move(current);
    }

    // 360-degree Gimbal-free Horizon Lock using Zero-Phase Gravity Vector
    const int horizonQuadrantDeg = detectGravityQuadrantDeg(frameGravityW[0]);
    result.detectedGravityQuadrantDeg = horizonQuadrantDeg;

    if (config.horizonLockEnabled && config.horizonLockAmount > 1e-4) {
        const double amount = std::min(1.0, std::max(0.0, config.horizonLockAmount));
        const double displayRotRad = static_cast<double>(horizonQuadrantDeg) * (kPi / 180.0);
        const double tiltRad = config.horizonTiltOffsetDeg * (kPi / 180.0);
        const Quat screenAlignQ = Quat::fromAxisAngle(Vec3(0.0, 0.0, 1.0), displayRotRad + tiltRad);

        for (size_t i = 0; i < n; ++i) {
            const Vec3 gW = frameGravityW[i].normalized();
            const Vec3 zc = result.smoothFrameQuats[i].rotateVector(Vec3(0.0, 0.0, 1.0)).normalized();
            Vec3 yc = gW - zc * gW.dot(zc);
            if (yc.normSquared() < 1e-6) {
                yc = result.smoothFrameQuats[i].rotateVector(Vec3(0.0, 1.0, 0.0));
            }
            yc = yc.normalized();
            const Vec3 xc = yc.cross(zc).normalized();

            const Quat qHorizon = (quatFromBasis(xc, yc, zc) * screenAlignQ).normalized();
            result.smoothFrameQuats[i] = slerpShortest(result.smoothFrameQuats[i], qHorizon, amount);
        }
    }

    // Light 5-tap (+-2 frames) Gaussian pass in so(3) before Swing-Lag Limiter
    if (n >= 5 && (pSmooth > 1e-4 || ySmooth > 1e-4 || rSmooth > 1e-4)) {
        std::vector<Quat> polished = result.smoothFrameQuats;
        constexpr double weights[5] = {1.0 / 16.0, 4.0 / 16.0, 6.0 / 16.0, 4.0 / 16.0, 1.0 / 16.0};
        for (size_t i = 2; i + 2 < n; ++i) {
            const Quat& qCenter = result.smoothFrameQuats[i];
            Vec3 rAvg(0.0, 0.0, 0.0);
            for (int k = -2; k <= 2; ++k) {
                const Vec3 rk = quatLog(qCenter.conjugate() * result.smoothFrameQuats[i + k]);
                rAvg = rAvg + rk * weights[k + 2];
            }
            polished[i] = (qCenter * quatExp(rAvg)).normalized();
        }
        result.smoothFrameQuats = std::move(polished);
    }

    // Pan-Adaptive Swing-Lag Limiter (Dynamic Shake-vs-Pan Excursion Ceiling)
    // Pure heavy shake (panRatio < 0.30, smoothness >= 0.80): knee=18.0 deg, maxCap=24.5 deg
    // Intentional pan (panRatio > 0.65): knee=8.0 deg, maxCap=12.5 deg
    const double smoothFactor = std::min(1.0, std::max(0.0, (std::max(pSmooth, ySmooth) - 0.50) / 0.30));
    const double zoomScale = (config.maxZoomLimit > 1.0 && config.maxZoomLimit < 1.45)
        ? std::max(0.45, (config.maxZoomLimit - 1.0) / 0.45)
        : 1.0;

    std::vector<double> kneeDegArr(n, 7.0);
    std::vector<double> maxCapDegArr(n, 11.0);
    const double targetShakeKnee = config.motoActionMode ? 19.5 : 18.0;
    const double targetShakeCap = config.motoActionMode ? 26.0 : 24.5;
    const double panStartRatio = config.motoActionMode ? 0.35 : 0.30;
    const double panSpanRatio = config.motoActionMode ? 0.30 : 0.35;
    for (size_t i = 0; i < n; ++i) {
        const double shakeKnee = 7.0 + smoothFactor * (targetShakeKnee - 7.0);
        const double shakeCap = 11.0 + smoothFactor * (targetShakeCap - 11.0);
        const double panKnee = 7.0 + smoothFactor * (8.0 - 7.0);
        const double panCap = 11.0 + smoothFactor * (12.5 - 11.0);

        const double tPan = std::min(1.0, std::max(0.0, (panRatioArr[i] - panStartRatio) / panSpanRatio));
        const double rawKnee = shakeKnee + tPan * (panKnee - shakeKnee);
        const double rawCap = shakeCap + tPan * (panCap - shakeCap);

        kneeDegArr[i] = 7.0 + zoomScale * (rawKnee - 7.0);
        maxCapDegArr[i] = 11.0 + zoomScale * (rawCap - 11.0);
    }
    smoothScalarGaussianTwoPass(kneeDegArr, 10.0);
    smoothScalarGaussianTwoPass(maxCapDegArr, 10.0);

    double maxSwingCap = 0.0;
    const Vec3 zAxis(0.0, 0.0, 1.0);
    for (size_t i = 0; i < n; ++i) {
        maxSwingCap = std::max(maxSwingCap, maxCapDegArr[i]);
        const double kneeRad = kneeDegArr[i] * (kPi / 180.0);
        const double headroomRad = std::max(0.5, maxCapDegArr[i] - kneeDegArr[i]) * (kPi / 180.0);

        const Quat qCorr = (result.rawFrameQuats[i].conjugate() * result.smoothFrameQuats[i]).normalized();
        Quat qSwing, qTwist;
        Quat::decomposeSwingTwist(qCorr, zAxis, qSwing, qTwist);

        if (qSwing.w < 0.0) {
            qSwing = Quat(-qSwing.w, -qSwing.x, -qSwing.y, -qSwing.z);
        }
        const double vNorm = std::sqrt(qSwing.x * qSwing.x + qSwing.y * qSwing.y + qSwing.z * qSwing.z);
        if (vNorm > 1e-8) {
            const double swingAngle = 2.0 * std::atan2(vNorm, std::min(1.0, qSwing.w));
            if (swingAngle > kneeRad) {
                const double compressedAngle = kneeRad + headroomRad * std::tanh((swingAngle - kneeRad) / headroomRad);
                const Vec3 axis(qSwing.x / vNorm, qSwing.y / vNorm, qSwing.z / vNorm);
                qSwing = Quat::fromAxisAngle(axis, compressedAngle);
                const Quat qCorrLimited = (qSwing * qTwist).normalized();
                result.smoothFrameQuats[i] = (result.rawFrameQuats[i] * qCorrLimited).normalized();
            }
        }
    }
    result.maxSwingCapDeg = maxSwingCap;

    result.rawAngularAccelRms = computeAngularAccelRms(result.rawFrameQuats, dtF);
    result.smoothAngularAccelRms = computeAngularAccelRms(result.smoothFrameQuats, dtF);

    if (result.rawAngularAccelRms > 1e-9) {
        const double rmsRatio = result.smoothAngularAccelRms / result.rawAngularAccelRms;
        result.stabilityImprovementPct = std::min(99.5, std::max(0.0, (1.0 - rmsRatio) * 100.0));
    }

    double maxInRoll = 0.0;
    double maxOutRoll = 0.0;
    double maxCorr = 0.0;
    for (size_t i = 0; i < n; ++i) {
        maxInRoll = std::max(maxInRoll, std::fabs(computeRollErrorDeg(result.rawFrameQuats[i], frameGravityW[i], horizonQuadrantDeg)));
        maxOutRoll = std::max(maxOutRoll, std::fabs(computeRollErrorDeg(result.smoothFrameQuats[i], frameGravityW[i], horizonQuadrantDeg)));
        const double corrDeg = rotationAngleBetween(result.rawFrameQuats[i], result.smoothFrameQuats[i]) * (180.0 / kPi);
        maxCorr = std::max(maxCorr, corrDeg);
    }
    result.maxInputRollDeg = maxInRoll;
    result.maxOutputRollErrorDeg = maxOutRoll;
    result.maxCorrectionDeg = maxCorr;

    result.valid = true;
    return result;
}

} // namespace gf
