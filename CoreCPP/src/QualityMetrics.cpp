#include "QualityMetrics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace gf {

namespace {

constexpr double kPi = 3.14159265358979323846;

Vec3 quatLogVec(const Quat& qIn) {
    Quat q = qIn.normalized();
    if (q.w < 0.0) {
        q = Quat(-q.w, -q.x, -q.y, -q.z);
    }
    const double vNorm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    if (vNorm < 1e-9) {
        return Vec3(2.0 * q.x, 2.0 * q.y, 2.0 * q.z);
    }
    const double theta = 2.0 * std::atan2(vNorm, q.w);
    const double s = theta / vNorm;
    return Vec3(q.x * s, q.y * s, q.z * s);
}

double computeAxisHighFreqEnergyAbove2Hz(const std::vector<double>& signal, double fs) {
    const size_t n = signal.size();
    if (n < 6 || fs <= 4.0) {
        return 0.0;
    }

    double meanVal = 0.0;
    for (double v : signal) {
        meanVal += v;
    }
    meanVal /= static_cast<double>(n);

    std::vector<double> windowed(n);
    double winEnergy = 0.0;
    for (size_t k = 0; k < n; ++k) {
        const double h = (n > 1)
            ? (0.5 * (1.0 - std::cos(2.0 * kPi * static_cast<double>(k) / static_cast<double>(n - 1))))
            : 1.0;
        windowed[k] = (signal[k] - meanVal) * h;
        winEnergy += h * h;
    }
    if (winEnergy <= 1e-12) {
        return 0.0;
    }

    const size_t half = n / 2;
    double powerSum = 0.0;
    for (size_t m = 1; m <= half; ++m) {
        const double freq = static_cast<double>(m) * fs / static_cast<double>(n);
        if (freq <= 2.0) {
            continue;
        }
        double re = 0.0;
        double im = 0.0;
        const double angleStep = 2.0 * kPi * static_cast<double>(m) / static_cast<double>(n);
        for (size_t k = 0; k < n; ++k) {
            const double ang = angleStep * static_cast<double>(k);
            re += windowed[k] * std::cos(ang);
            im -= windowed[k] * std::sin(ang);
        }
        const double magSq = re * re + im * im;
        const double oneSidedFactor = (2 * m < n) ? 2.0 : 1.0;
        powerSum += (oneSidedFactor * magSq) / (static_cast<double>(n) * winEnergy);
    }
    return powerSum;
}

double computeAxisLiuStabilityScore(const std::vector<double>& omegaAxis, double dt) {
    const size_t m = omegaAxis.size();
    if (m < 4) {
        return 1.0;
    }

    const size_t n = m + 1;
    std::vector<double> path(n, 0.0);
    for (size_t i = 0; i < m; ++i) {
        path[i + 1] = path[i] + omegaAxis[i] * dt;
    }

    // Endpoint linear detrend eliminates boundary jump leakage on intentional pans
    const double p0 = path.front();
    const double pEnd = path.back();
    double meanDetrended = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double u = static_cast<double>(i) / static_cast<double>(n - 1);
        path[i] -= p0 + u * (pEnd - p0);
        meanDetrended += path[i];
    }
    meanDetrended /= static_cast<double>(n);
    for (size_t i = 0; i < n; ++i) {
        path[i] -= meanDetrended;
    }

    const size_t half = n / 2;
    if (half == 0) {
        return 1.0;
    }

    double totalEnergy = 0.0;
    double low6Energy = 0.0;
    const size_t lowBins = std::min<size_t>(6, half);

    for (size_t k = 1; k <= half; ++k) {
        double re = 0.0;
        double im = 0.0;
        const double angleStep = 2.0 * kPi * static_cast<double>(k) / static_cast<double>(n);
        for (size_t t = 0; t < n; ++t) {
            const double ang = angleStep * static_cast<double>(t);
            re += path[t] * std::cos(ang);
            im -= path[t] * std::sin(ang);
        }
        const double weight = (2 * k < n) ? 2.0 : 1.0;
        const double binPower = weight * (re * re + im * im);
        totalEnergy += binPower;
        if (k <= lowBins) {
            low6Energy += binPower;
        }
    }

    const double rmsRad = std::sqrt(totalEnergy / (static_cast<double>(n) * static_cast<double>(n)));
    const double rawScore = (totalEnergy > 1e-12) ? std::min(1.0, std::max(0.0, low6Energy / totalEnergy)) : 1.0;

    if (rmsRad <= 0.0015) {
        return 1.0;
    }
    if (rmsRad < 0.0045) {
        const double u = (rmsRad - 0.0015) / (0.0045 - 0.0015);
        const double w = u * u * (3.0 - 2.0 * u);
        return (1.0 - w) * 1.0 + w * rawScore;
    }
    return rawScore;
}

} // namespace

std::vector<Vec3> QualityMetrics::computeRelativeAngularVelocities(
    const std::vector<Quat>& quats,
    double nominalFps
) {
    std::vector<Vec3> omega;
    if (quats.size() < 2) {
        return omega;
    }
    const double fps = (nominalFps > 1.0) ? nominalFps : 30.0;
    const double dt = 1.0 / fps;
    omega.reserve(quats.size() - 1);
    for (size_t i = 0; i + 1 < quats.size(); ++i) {
        const Quat dq = (quats[i].conjugate() * quats[i + 1]).normalized();
        omega.push_back(quatLogVec(dq) * (1.0 / dt));
    }
    return omega;
}

QualityReport QualityMetrics::evaluateOmegaSequence(
    const std::vector<Vec3>& omegaRes,
    double nominalFps,
    const std::vector<double>* zoomUsed,
    double maxHorizonErrorDeg,
    double accelJumpThresholdRadPerSec2
) {
    QualityReport r;
    const double fps = (nominalFps > 1.0) ? nominalFps : 30.0;
    const double dt = 1.0 / fps;
    r.nominalFps = fps;
    r.frameCount = omegaRes.empty() ? 0 : (omegaRes.size() + 1);
    r.maxHorizonErrorDeg = maxHorizonErrorDeg;
    r.accelJumpThresholdRadPerSec2 = accelJumpThresholdRadPerSec2;

    if (zoomUsed && !zoomUsed->empty()) {
        double cropSum = 0.0;
        double zoomSum = 0.0;
        double maxZ = 1.0;
        double minCrop = 1.0;
        for (double zRaw : *zoomUsed) {
            const double z = std::max(1.0, zRaw);
            const double crop = 1.0 / z;
            cropSum += crop;
            zoomSum += z;
            maxZ = std::max(maxZ, z);
            minCrop = std::min(minCrop, crop);
        }
        const double cnt = static_cast<double>(zoomUsed->size());
        r.meanCroppingRatio = cropSum / cnt;
        r.minCroppingRatio = minCrop;
        r.avgZoom = zoomSum / cnt;
        r.maxZoom = maxZ;
    }

    const size_t m = omegaRes.size();
    if (m < 3) {
        return r;
    }

    std::vector<double> wx(m), wy(m), wz(m);
    double omegaSqSum = 0.0;
    for (size_t i = 0; i < m; ++i) {
        wx[i] = omegaRes[i].x;
        wy[i] = omegaRes[i].y;
        wz[i] = omegaRes[i].z;
        omegaSqSum += omegaRes[i].normSquared();
    }
    r.visualOmegaResRms = std::sqrt(omegaSqSum / static_cast<double>(m));

    const double eX = computeAxisHighFreqEnergyAbove2Hz(wx, fps);
    const double eY = computeAxisHighFreqEnergyAbove2Hz(wy, fps);
    const double eZ = computeAxisHighFreqEnergyAbove2Hz(wz, fps);
    r.highFreqOmegaEnergyAbove2Hz = eX + eY + eZ;
    r.highFreqOmegaRmsAbove2Hz = std::sqrt(std::max(0.0, r.highFreqOmegaEnergyAbove2Hz));

    const double liuX = computeAxisLiuStabilityScore(wx, dt);
    const double liuY = computeAxisLiuStabilityScore(wy, dt);
    const double liuZ = computeAxisLiuStabilityScore(wz, dt);
    r.liuStabilityScore = std::min({liuX, liuY, liuZ});
    r.liuStabilityScoreMean = (liuX + liuY + liuZ) / 3.0;

    std::vector<Vec3> accel(m - 1);
    for (size_t i = 0; i + 1 < m; ++i) {
        accel[i] = (omegaRes[i + 1] - omegaRes[i]) * (1.0 / dt);
    }

    double jerkSqSum = 0.0;
    size_t jerkCount = 0;
    int jumpCount = 0;
    double maxJump = 0.0;
    for (size_t i = 1; i < accel.size(); ++i) {
        const Vec3 da = accel[i] - accel[i - 1];
        const double jumpMag = da.norm();
        maxJump = std::max(maxJump, jumpMag);
        if (jumpMag > accelJumpThresholdRadPerSec2) {
            ++jumpCount;
        }
        const Vec3 j = da * (1.0 / dt);
        jerkSqSum += j.normSquared();
        ++jerkCount;
    }

    r.jerkRms = (jerkCount > 0) ? std::sqrt(jerkSqSum / static_cast<double>(jerkCount)) : 0.0;
    r.accelDiscontinuityCount = jumpCount;
    r.maxAccelJumpRadPerSec2 = maxJump;
    return r;
}

QualityReport QualityMetrics::evaluateTrajectory(
    const std::vector<Quat>& smoothQuats,
    double nominalFps,
    const std::vector<double>* zoomUsed,
    double maxHorizonErrorDeg,
    double accelJumpThresholdRadPerSec2
) {
    const std::vector<Vec3> omega = computeRelativeAngularVelocities(smoothQuats, nominalFps);
    return evaluateOmegaSequence(omega, nominalFps, zoomUsed, maxHorizonErrorDeg, accelJumpThresholdRadPerSec2);
}

PtsCheckResult QualityMetrics::validatePtsIntegrity(
    const std::vector<double>& inputPtsSec,
    const std::vector<double>& outputPtsSec,
    double nominalFps
) {
    PtsCheckResult res;
    res.inputCount = inputPtsSec.size();
    res.outputCount = outputPtsSec.size();

    if (inputPtsSec.size() > outputPtsSec.size()) {
        res.writerDroppedCount = inputPtsSec.size() - outputPtsSec.size();
    }

    if (outputPtsSec.size() < 2) {
        res.passed = (!outputPtsSec.empty() && outputPtsSec.size() == inputPtsSec.size());
        return res;
    }

    const double fps = (nominalFps > 1.0) ? nominalFps : 30.0;
    const double dupThreshSec = 0.35 / fps;

    std::vector<double> dts;
    dts.reserve(outputPtsSec.size() - 1);
    double minDt = 1e18;
    double maxDt = -1e18;
    double sumDt = 0.0;

    for (size_t i = 1; i < outputPtsSec.size(); ++i) {
        const double dt = outputPtsSec[i] - outputPtsSec[i - 1];
        if (dt <= 1e-6) {
            ++res.nonMonotonicCount;
        } else if (dt < dupThreshSec) {
            ++res.duplicateFrameCount;
        }
        dts.push_back(dt);
        minDt = std::min(minDt, dt);
        maxDt = std::max(maxDt, dt);
        sumDt += dt;
    }

    const double meanDt = sumDt / static_cast<double>(dts.size());
    double varDt = 0.0;
    for (double dt : dts) {
        const double d = dt - meanDt;
        varDt += d * d;
    }
    res.minDtMs = minDt * 1000.0;
    res.maxDtMs = maxDt * 1000.0;
    res.meanDtMs = meanDt * 1000.0;
    res.stdDevDtMs = std::sqrt(varDt / static_cast<double>(dts.size())) * 1000.0;

    double maxDrift = 0.0;
    const size_t minCount = std::min(inputPtsSec.size(), outputPtsSec.size());
    for (size_t i = 1; i < minCount; ++i) {
        const double inDt = inputPtsSec[i] - inputPtsSec[i - 1];
        const double outDt = outputPtsSec[i] - outputPtsSec[i - 1];
        maxDrift = std::max(maxDrift, std::fabs(outDt - inDt) * 1000.0);
    }
    res.maxInputOutputStepDriftMs = maxDrift;

    res.passed = (res.inputCount == res.outputCount) &&
                 (res.nonMonotonicCount == 0) &&
                 (res.duplicateFrameCount == 0) &&
                 (res.writerDroppedCount == 0) &&
                 (res.maxInputOutputStepDriftMs < 1.0);
    return res;
}

std::vector<double> QualityMetrics::lowPassFilterZoomSub1Hz(
    const std::vector<double>& zoomCurve,
    double fps,
    double cutoffHz
) {
    std::vector<double> out = zoomCurve;
    const size_t n = out.size();
    const double fs = (fps > 1.0) ? fps : 30.0;
    const double fc = std::min(0.95, std::max(0.10, cutoffHz));
    if (n < 6 || fs <= 2.2 * fc) {
        return out;
    }

    const double w0 = 2.0 * kPi * fc / fs;
    const double cosW0 = std::cos(w0);
    const double sinW0 = std::sin(w0);
    const double alpha = sinW0 / std::sqrt(2.0);
    const double a0 = 1.0 + alpha;
    const double b0 = ((1.0 - cosW0) * 0.5) / a0;
    const double b1 = (1.0 - cosW0) / a0;
    const double b2 = b0;
    const double a1 = (-2.0 * cosW0) / a0;
    const double a2 = (1.0 - alpha) / a0;

    auto runPass = [&](bool reverse) {
        auto getAt = [&](size_t idx) -> double { return reverse ? out[n - 1 - idx] : out[idx]; };
        auto setAt = [&](size_t idx, double v) {
            if (reverse) {
                out[n - 1 - idx] = v;
            } else {
                out[idx] = v;
            }
        };
        double x1 = getAt(0), x2 = getAt(0);
        double y1 = getAt(0), y2 = getAt(0);
        for (size_t i = 0; i < n; ++i) {
            const double x0 = getAt(i);
            const double y0 = b0 * x0 + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
            setAt(i, y0);
            x2 = x1; x1 = x0;
            y2 = y1; y1 = y0;
        }
    };

    runPass(false);
    runPass(true);
    for (double& z : out) {
        z = std::max(1.0, z);
    }
    return out;
}

std::string QualityMetrics::ptsToJson(const PtsCheckResult& pts) {
    char buf[512];
    std::snprintf(
        buf, sizeof(buf),
        "{\"inputCount\":%zu,\"outputCount\":%zu,\"nonMonotonicCount\":%zu,"
        "\"duplicateFrameCount\":%zu,\"writerDroppedCount\":%zu,"
        "\"minDtMs\":%.4f,\"maxDtMs\":%.4f,\"meanDtMs\":%.4f,\"stdDevDtMs\":%.4f,"
        "\"maxInputOutputStepDriftMs\":%.4f,\"passed\":%s}",
        pts.inputCount, pts.outputCount, pts.nonMonotonicCount,
        pts.duplicateFrameCount, pts.writerDroppedCount,
        pts.minDtMs, pts.maxDtMs, pts.meanDtMs, pts.stdDevDtMs,
        pts.maxInputOutputStepDriftMs, pts.passed ? "true" : "false"
    );
    return std::string(buf);
}

std::string QualityMetrics::toJson(const QualityReport& r, const std::string& clipName) {
    std::ostringstream oss;
    char buf[1600];
    std::snprintf(
        buf, sizeof(buf),
        "{\n"
        "  \"clipName\": \"%s\",\n"
        "  \"frameCount\": %zu,\n"
        "  \"nominalFps\": %.3f,\n"
        "  \"hasVisualMetrics\": %s,\n"
        "  \"highFreqOmegaEnergyAbove2Hz\": %.6f,\n"
        "  \"highFreqOmegaRmsAbove2Hz\": %.6f,\n"
        "  \"visualOmegaResRms\": %.6f,\n"
        "  \"jerkRms\": %.6f,\n"
        "  \"liuStabilityScore\": %.6f,\n"
        "  \"liuStabilityScoreMean\": %.6f,\n"
        "  \"meanCroppingRatio\": %.6f,\n"
        "  \"minCroppingRatio\": %.6f,\n"
        "  \"avgZoom\": %.6f,\n"
        "  \"maxZoom\": %.6f,\n"
        "  \"accelDiscontinuityCount\": %d,\n"
        "  \"maxAccelJumpRadPerSec2\": %.6f,\n"
        "  \"accelJumpThresholdRadPerSec2\": %.3f,\n"
        "  \"maxHorizonErrorDeg\": %.6f,\n"
        "  \"meanInlierRatio\": %.6f,\n"
        "  \"calibResidualRmsBefore\": %.6f,\n"
        "  \"calibResidualRmsAfter\": %.6f,\n"
        "  \"ptsCheck\": %s\n"
        "}",
        clipName.c_str(),
        r.frameCount,
        r.nominalFps,
        r.hasVisualMetrics ? "true" : "false",
        r.highFreqOmegaEnergyAbove2Hz,
        r.highFreqOmegaRmsAbove2Hz,
        r.visualOmegaResRms,
        r.jerkRms,
        r.liuStabilityScore,
        r.liuStabilityScoreMean,
        r.meanCroppingRatio,
        r.minCroppingRatio,
        r.avgZoom,
        r.maxZoom,
        r.accelDiscontinuityCount,
        r.maxAccelJumpRadPerSec2,
        r.accelJumpThresholdRadPerSec2,
        r.maxHorizonErrorDeg,
        r.meanInlierRatio,
        r.calibResidualRmsBefore,
        r.calibResidualRmsAfter,
        ptsToJson(r.ptsCheck).c_str()
    );
    return std::string(buf);
}

VideoQualityAnalyzer::VideoQualityAnalyzer(const VirtualGyroConfig& config)
    : config_(config), virtualGyro_(config) {
}

bool VideoQualityAnalyzer::pushRenderedLumaFrame(const uint8_t* luma, int width, int height, double ptsSec) {
    return virtualGyro_.pushLumaFrame(luma, width, height, ptsSec);
}

QualityReport VideoQualityAnalyzer::finalize(
    const std::vector<double>* zoomUsed,
    double maxHorizonErrorDeg,
    double accelJumpThresholdRadPerSec2
) const {
    const auto& samples = virtualGyro_.estimatedSamples();
    std::vector<Vec3> omega;
    omega.reserve(samples.size());
    for (const auto& s : samples) {
        omega.emplace_back(s.wxCam, s.wyCam, s.wzCam);
    }
    QualityReport report = QualityMetrics::evaluateOmegaSequence(
        omega, config_.nominalFps, zoomUsed, maxHorizonErrorDeg, accelJumpThresholdRadPerSec2
    );
    report.frameCount = virtualGyro_.frames().size();
    report.meanInlierRatio = virtualGyro_.meanInlierRatio();
    report.hasVisualMetrics = !samples.empty();
    return report;
}

} // namespace gf
