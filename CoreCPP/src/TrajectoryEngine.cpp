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
    // Columns of R_world_from_cam: [right, down, forward]
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

double computeAlphaForAxis(double smoothness, double smoothOmega, double velocityDamping, double dtF) {
    const double s = std::min(1.0, std::max(0.0, smoothness));
    if (s <= 1e-4) {
        return 0.0;
    }
    const double tauBase = 0.02 + s * 0.88;
    const double ratio = smoothOmega / 0.55;
    const double tauEff = tauBase / (1.0 + velocityDamping * ratio * ratio);
    return std::exp(-dtF / std::max(tauEff, 1e-4));
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

    initGravityCam_ = Vec3(0.0, 1.0, 0.0);
    if (!sortedAccel.empty()) {
        const size_t initCount = std::min<size_t>(20, sortedAccel.size());
        Vec3 sumA(0.0, 0.0, 0.0);
        for (size_t i = 0; i < initCount; ++i) {
            const Vec3 aIos(sortedAccel[i].ax, sortedAccel[i].ay, sortedAccel[i].az);
            sumA = sumA + applyAxisMatrix(M, aIos);
        }
        if (sumA.normSquared() > 1e-6) {
            initGravityCam_ = sumA.normalized();
        }
    }

    const Vec3 biasIos(header.gyro_bias_x, header.gyro_bias_y, header.gyro_bias_z);
    const double kpMahony = 0.5;

    integratedSamples_.reserve(dedupGyro.size());
    Quat q = Quat::identity();
    size_t accelIdx = 0;

    for (size_t i = 0; i < dedupGyro.size(); ++i) {
        const auto& g = dedupGyro[i];
        const Vec3 rawOmegaIos(g.gx - biasIos.x, g.gy - biasIos.y, g.gz - biasIos.z);
        Vec3 omegaCam = applyAxisMatrix(M, rawOmegaIos);

        if (i > 0) {
            const double dt = std::min(0.1, std::max(1e-6, g.timestamp_sec - dedupGyro[i - 1].timestamp_sec));

            Vec3 feedback(0.0, 0.0, 0.0);
            if (!sortedAccel.empty()) {
                while (accelIdx + 1 < sortedAccel.size() &&
                       std::fabs(sortedAccel[accelIdx + 1].timestamp_sec - g.timestamp_sec) <=
                       std::fabs(sortedAccel[accelIdx].timestamp_sec - g.timestamp_sec)) {
                    ++accelIdx;
                }
                const auto& a = sortedAccel[accelIdx];
                if (std::fabs(a.timestamp_sec - g.timestamp_sec) < 0.05) {
                    const Vec3 aCam = applyAxisMatrix(M, Vec3(a.ax, a.ay, a.az));
                    const double aNorm = aCam.norm();
                    if (aNorm >= 0.85 && aNorm <= 1.15) {
                        const Vec3 aMeas = aCam * (1.0 / aNorm);
                        const Vec3 gEst = q.conjugate().rotateVector(initGravityCam_);
                        feedback = aMeas.cross(gEst) * kpMahony;
                    }
                }
            }

            q = Quat::integrateGyroStep(q, omegaCam + feedback, dt);
        }

        integratedSamples_.push_back({g.timestamp_sec, q, omegaCam});
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

    const double offsetSec = config.timeOffsetMs * 0.001;
    for (size_t i = 0; i < n; ++i) {
        const double tMid = frames[i].pts_sec - 0.5 * static_cast<double>(frames[i].exposure_duration_sec) + offsetSec;
        result.frameTimestampsSec[i] = tMid;
        result.rawFrameQuats[i] = sampleRawQuaternionAt(tMid);
    }

    if (n < 3) {
        result.smoothFrameQuats = result.rawFrameQuats;
        result.valid = true;
        return result;
    }

    const double dtF = 1.0 / nominalFps_;
    std::vector<double> omegaMag(n, 0.0);
    for (size_t i = 1; i < n; ++i) {
        const double angle = rotationAngleBetween(result.rawFrameQuats[i - 1], result.rawFrameQuats[i]);
        omegaMag[i] = angle / dtF;
    }
    omegaMag[0] = omegaMag[1];

    const int radius = std::max(1, static_cast<int>(std::round(0.15 * nominalFps_)));
    std::vector<double> smoothOmega(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        const int lo = std::max(0, static_cast<int>(i) - radius);
        const int hi = std::min(static_cast<int>(n) - 1, static_cast<int>(i) + radius);
        double sum = 0.0;
        for (int k = lo; k <= hi; ++k) {
            sum += omegaMag[static_cast<size_t>(k)];
        }
        smoothOmega[i] = sum / static_cast<double>(hi - lo + 1);
    }

    const double pSmooth = config.pitchSmoothness;
    const double ySmooth = config.yawSmoothness;
    const double rSmooth = config.rollSmoothness;

    if (pSmooth <= 1e-4 && ySmooth <= 1e-4 && rSmooth <= 1e-4 && !config.horizonLockEnabled) {
        result.smoothFrameQuats = result.rawFrameQuats;
        result.rawAngularAccelRms = computeAngularAccelRms(result.rawFrameQuats, dtF);
        result.smoothAngularAccelRms = result.rawAngularAccelRms;
        result.stabilityImprovementPct = 0.0;
        result.valid = true;
        return result;
    }

    // Forward pass in so(3) tangent space for independent Pitch/Yaw/Roll filtering
    std::vector<Quat> qFwd(n);
    qFwd[0] = result.rawFrameQuats[0];
    for (size_t i = 1; i < n; ++i) {
        const double aPitch = computeAlphaForAxis(pSmooth, smoothOmega[i], config.velocityDamping, dtF);
        const double aYaw = computeAlphaForAxis(ySmooth, smoothOmega[i], config.velocityDamping, dtF);
        const double aRoll = computeAlphaForAxis(rSmooth, smoothOmega[i], config.velocityDamping, dtF);

        const Vec3 rDiff = quatLog(qFwd[i - 1].conjugate() * result.rawFrameQuats[i]);
        const Vec3 rStep(
            rDiff.x * (1.0 - aPitch),
            rDiff.y * (1.0 - aYaw),
            rDiff.z * (1.0 - aRoll)
        );
        qFwd[i] = (qFwd[i - 1] * quatExp(rStep)).normalized();
    }

    // Backward pass in so(3) tangent space (zero phase lag)
    result.smoothFrameQuats[n - 1] = qFwd[n - 1];
    for (size_t i = n - 1; i > 0; --i) {
        const size_t idx = i - 1;
        const double aPitch = computeAlphaForAxis(pSmooth, smoothOmega[idx], config.velocityDamping, dtF);
        const double aYaw = computeAlphaForAxis(ySmooth, smoothOmega[idx], config.velocityDamping, dtF);
        const double aRoll = computeAlphaForAxis(rSmooth, smoothOmega[idx], config.velocityDamping, dtF);

        const Vec3 rDiff = quatLog(result.smoothFrameQuats[idx + 1].conjugate() * qFwd[idx]);
        const Vec3 rStep(
            rDiff.x * (1.0 - aPitch),
            rDiff.y * (1.0 - aYaw),
            rDiff.z * (1.0 - aRoll)
        );
        result.smoothFrameQuats[idx] = (result.smoothFrameQuats[idx + 1] * quatExp(rStep)).normalized();
    }

    // 360-degree Gimbal-free Horizon Lock around world gravity vector
    if (config.horizonLockEnabled && config.horizonLockAmount > 1e-4) {
        const double amount = std::min(1.0, std::max(0.0, config.horizonLockAmount));
        const double tiltRad = config.horizonTiltOffsetDeg * (kPi / 180.0);
        const Quat tiltQ = Quat::fromAxisAngle(Vec3(0.0, 0.0, 1.0), tiltRad);
        const Vec3 gWorld = initGravityCam_.normalized();

        for (size_t i = 0; i < n; ++i) {
            const Vec3 forwardW = result.smoothFrameQuats[i].rotateVector(Vec3(0.0, 0.0, 1.0)).normalized();
            Vec3 rightW = gWorld.cross(forwardW);
            if (rightW.normSquared() < 1e-6) {
                rightW = result.smoothFrameQuats[i].rotateVector(Vec3(1.0, 0.0, 0.0));
            }
            rightW = rightW.normalized();
            const Vec3 downW = forwardW.cross(rightW).normalized();

            const Quat qHorizon = (quatFromBasis(rightW, downW, forwardW) * tiltQ).normalized();
            result.smoothFrameQuats[i] = slerpShortest(result.smoothFrameQuats[i], qHorizon, amount);
        }
    }

    result.rawAngularAccelRms = computeAngularAccelRms(result.rawFrameQuats, dtF);
    result.smoothAngularAccelRms = computeAngularAccelRms(result.smoothFrameQuats, dtF);

    if (result.rawAngularAccelRms > 1e-9) {
        const double rmsRatio = result.smoothAngularAccelRms / result.rawAngularAccelRms;
        result.stabilityImprovementPct = std::min(99.5, std::max(0.0, (1.0 - rmsRatio) * 100.0));
    }
    result.valid = true;
    return result;
}

} // namespace gf
