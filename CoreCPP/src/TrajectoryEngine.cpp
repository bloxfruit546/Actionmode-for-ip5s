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
    // Cinema-grade tauBase: ~1.28s at s=0.65, ~2.53s at s=0.85, 4.00s at s=1.0
    const double tauBase = 0.02 + 0.90 * s + 3.08 * std::pow(s, 3.5);
    // Only reduce tau during genuine intentional panning (panActivity > 0)
    const double tauEff = tauBase / (1.0 + velocityDamping * panActivity * 4.0);
    return std::exp(-dtF / std::max(tauEff, 1e-4));
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

} // namespace

void TrajectoryEngine::setImuData(
    const GFFileHeader& header,
    const std::vector<GFGyroRecord>& gyroRecords,
    const std::vector<GFAccelRecord>& accelRecords,
    const double* customAxisMatrix
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

    // Pass 1: Pure RK4 Gyro integration (immune to linear shake acceleration)
    const size_t nGyro = dedupGyro.size();
    std::vector<Quat> qPureGyro(nGyro, Quat::identity());
    std::vector<Vec3> omegaCamArr(nGyro);

    for (size_t i = 0; i < nGyro; ++i) {
        const auto& g = dedupGyro[i];
        const Vec3 rawOmegaIos(g.gx - biasIos.x, g.gy - biasIos.y, g.gz - biasIos.z);
        omegaCamArr[i] = applyAxisMatrix(M, rawOmegaIos);
        if (i > 0) {
            const double dt = std::min(0.1, std::max(1e-6, g.timestamp_sec - dedupGyro[i - 1].timestamp_sec));
            qPureGyro[i] = Quat::integrateGyroStep(qPureGyro[i - 1], omegaCamArr[i], dt);
        }
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
            double w = 0.001;
            if (magDev <= 0.08 && jerk <= 4.0) {
                const double wMag = (magDev <= 0.03) ? 1.0 : (1.0 - (magDev - 0.03) / 0.05);
                const double wJerk = (jerk <= 1.5) ? 1.0 : (1.0 - (jerk - 1.5) / 2.5);
                w = std::max(0.001, wMag * wJerk);
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

        // Forward-Backward Zero-Phase Weighted LPF (tau_g = 3.0s)
        const double tauG = 3.0;
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
        const double kpMahony = 0.35;

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
                if (accWeight[aIdx] > 0.2 && std::fabs(sortedAccel[aIdx].timestamp_sec - tG) < 0.05) {
                    const Vec3 aCam = applyAxisMatrix(M, Vec3(sortedAccel[aIdx].ax, sortedAccel[aIdx].ay, sortedAccel[aIdx].az)).normalized();
                    const Vec3 gEst = q.conjugate().rotateVector(gSmooth[aIdx]);
                    feedback = aCam.cross(gEst) * (kpMahony * accWeight[aIdx]);
                }
                q = Quat::integrateGyroStep(q, omegaCamArr[i] + feedback, dt);
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
    const double t = (timestampSec - s0.timestampSec) / span;
    return slerpShortest(s0.quat, s1.quat, t);
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
        const double tMid = frames[i].pts_sec - 0.5 * static_cast<double>(frames[i].exposure_duration_sec) + offsetSec;
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

    // True Net-Displacement Pan Detector over +-0.45s window:
    // panRatio = D_i / (L_i + 1e-4). Only damp tau when panRatio > 0.65 AND D_i > 12 deg.
    const int radius = std::max(2, static_cast<int>(std::round(0.45 * nominalFps_)));
    std::vector<double> stepAngles(n, 0.0);
    for (size_t i = 1; i < n; ++i) {
        stepAngles[i] = rotationAngleBetween(result.rawFrameQuats[i - 1], result.rawFrameQuats[i]);
    }

    std::vector<double> panActivity(n, 0.0);
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
        panRatioSum += ratio;

        if (ratio > 0.65 && netDisp > minNetDispRad) {
            panActivity[i] = std::min(1.0, (ratio - 0.65) / 0.35);
        }
    }
    result.avgPanRatio = panRatioSum / static_cast<double>(n);

    const double pSmooth = config.pitchSmoothness;
    const double ySmooth = config.yawSmoothness;
    const double rSmooth = config.rollSmoothness;

    if (pSmooth <= 1e-4 && ySmooth <= 1e-4 && rSmooth <= 1e-4 && !config.horizonLockEnabled) {
        result.smoothFrameQuats = result.rawFrameQuats;
    } else {
        // 3-Pass Forward-Backward Zero-Phase Smoother (Gaussian-like C^3 smoothness)
        std::vector<Quat> current = result.rawFrameQuats;
        std::vector<Quat> qFwd(n);
        std::vector<Quat> qBwd(n);

        for (int pass = 0; pass < 3; ++pass) {
            qFwd[0] = current[0];
            for (size_t i = 1; i < n; ++i) {
                const double aPitch = computeAlphaForAxis(pSmooth, panActivity[i], config.velocityDamping, dtF);
                const double aYaw = computeAlphaForAxis(ySmooth, panActivity[i], config.velocityDamping, dtF);
                const double aRoll = computeAlphaForAxis(rSmooth, panActivity[i], config.velocityDamping, dtF);

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
                const double aPitch = computeAlphaForAxis(pSmooth, panActivity[idx], config.velocityDamping, dtF);
                const double aYaw = computeAlphaForAxis(ySmooth, panActivity[idx], config.velocityDamping, dtF);
                const double aRoll = computeAlphaForAxis(rSmooth, panActivity[idx], config.velocityDamping, dtF);

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
