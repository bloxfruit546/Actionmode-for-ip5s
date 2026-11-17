#include "TrajectoryEngine.hpp"

#include <algorithm>
#include <cmath>

namespace gf {

namespace {

Quat slerpShortest(const Quat& a, const Quat& b, double t) {
    return Quat::slerp(a, b, t);
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

} // namespace

void TrajectoryEngine::setImuData(
    const GFFileHeader& header,
    const std::vector<GFGyroRecord>& gyroRecords,
    const std::vector<GFAccelRecord>& accelRecords
) {
    integratedSamples_.clear();
    nominalFps_ = (header.nominal_fps > 1.0) ? header.nominal_fps : 30.0;

    if (gyroRecords.empty()) {
        return;
    }

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

    // Estimate initial gravity vector in camera frame from first 20 accel samples
    Vec3 initGravityCam(0.0, 1.0, 0.0);
    if (!sortedAccel.empty()) {
        const size_t initCount = std::min<size_t>(20, sortedAccel.size());
        Vec3 sumA(0.0, 0.0, 0.0);
        for (size_t i = 0; i < initCount; ++i) {
            const Vec3 aIos(sortedAccel[i].ax, sortedAccel[i].ay, sortedAccel[i].az);
            sumA = sumA + mapIOSImuToCameraFrame(aIos);
        }
        if (sumA.normSquared() > 1e-6) {
            initGravityCam = sumA.normalized();
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
        Vec3 omegaCam = mapIOSImuToCameraFrame(rawOmegaIos);

        if (i > 0) {
            const double dt = std::min(0.1, std::max(1e-6, g.timestamp_sec - dedupGyro[i - 1].timestamp_sec));

            // Find nearest accel sample for Mahony gravity fusion
            Vec3 feedback(0.0, 0.0, 0.0);
            if (!sortedAccel.empty()) {
                while (accelIdx + 1 < sortedAccel.size() &&
                       std::fabs(sortedAccel[accelIdx + 1].timestamp_sec - g.timestamp_sec) <=
                       std::fabs(sortedAccel[accelIdx].timestamp_sec - g.timestamp_sec)) {
                    ++accelIdx;
                }
                const auto& a = sortedAccel[accelIdx];
                if (std::fabs(a.timestamp_sec - g.timestamp_sec) < 0.05) {
                    const Vec3 aCam = mapIOSImuToCameraFrame(Vec3(a.ax, a.ay, a.az));
                    const double aNorm = aCam.norm();
                    if (aNorm >= 0.85 && aNorm <= 1.15) {
                        const Vec3 aMeas = aCam * (1.0 / aNorm);
                        const Vec3 gEst = q.conjugate().rotateVector(initGravityCam);
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

    // Boundary clamping / bounded extrapolation (<= 100ms) for Sync Coverage: WARN takes
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

    // Smooth angular velocity magnitude (~0.3s boxcar) to distinguish intentional pan from hand jitter
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

    const double sClamped = std::min(1.0, std::max(0.0, config.smoothness));
    if (sClamped <= 1e-4) {
        result.smoothFrameQuats = result.rawFrameQuats;
        result.rawAngularAccelRms = computeAngularAccelRms(result.rawFrameQuats, dtF);
        result.smoothAngularAccelRms = result.rawAngularAccelRms;
        result.stabilityImprovementPct = 0.0;
        result.valid = true;
        return result;
    }

    // At smoothness = 0.65, tauBase ~= 0.59s (within 0.45s - 0.65s target range)
    const double tauBase = 0.02 + sClamped * 0.88;

    std::vector<double> alpha(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        const double ratio = smoothOmega[i] / 0.55;
        const double tauEff = tauBase / (1.0 + config.velocityDamping * ratio * ratio);
        alpha[i] = std::exp(-dtF / std::max(tauEff, 1e-4));
    }

    // Forward pass
    std::vector<Quat> qFwd(n);
    qFwd[0] = result.rawFrameQuats[0];
    for (size_t i = 1; i < n; ++i) {
        qFwd[i] = slerpShortest(result.rawFrameQuats[i], qFwd[i - 1], alpha[i]);
    }

    // Backward pass (zero phase lag)
    result.smoothFrameQuats[n - 1] = qFwd[n - 1];
    for (size_t i = n - 1; i > 0; --i) {
        const size_t idx = i - 1;
        result.smoothFrameQuats[idx] = slerpShortest(qFwd[idx], result.smoothFrameQuats[idx + 1], alpha[idx]);
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
