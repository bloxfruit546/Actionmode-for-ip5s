#include "CalibrationRefiner.hpp"
#include "Quaternion.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace gf {

namespace {

bool extractJsonNumber(const std::string& text, const std::string& key, double& outVal) {
    const std::string token = "\"" + key + "\"";
    const size_t pos = text.find(token);
    if (pos == std::string::npos) return false;
    const size_t colon = text.find(':', pos + token.size());
    if (colon == std::string::npos) return false;
    const char* start = text.c_str() + colon + 1;
    char* endPtr = nullptr;
    const double val = std::strtod(start, &endPtr);
    if (endPtr == start || !std::isfinite(val)) return false;
    outVal = val;
    return true;
}

bool extractJsonBool(const std::string& text, const std::string& key, bool& outVal) {
    const std::string token = "\"" + key + "\"";
    const size_t pos = text.find(token);
    if (pos == std::string::npos) return false;
    const size_t colon = text.find(':', pos + token.size());
    if (colon == std::string::npos) return false;
    const size_t tPos = text.find("true", colon + 1);
    const size_t fPos = text.find("false", colon + 1);
    const size_t commaOrBrace = text.find_first_of(",}\n", colon + 1);
    if (tPos != std::string::npos && (commaOrBrace == std::string::npos || tPos < commaOrBrace)) {
        outVal = true;
        return true;
    }
    if (fPos != std::string::npos && (commaOrBrace == std::string::npos || fPos < commaOrBrace)) {
        outVal = false;
        return true;
    }
    return false;
}

bool extractJsonArray(const std::string& text, const std::string& key, double* outArr, size_t count) {
    const std::string token = "\"" + key + "\"";
    const size_t pos = text.find(token);
    if (pos == std::string::npos) return false;
    const size_t lb = text.find('[', pos + token.size());
    const size_t rb = text.find(']', lb == std::string::npos ? pos : lb);
    if (lb == std::string::npos || rb == std::string::npos || rb <= lb) return false;
    const std::string sub = text.substr(lb + 1, rb - lb - 1);
    const char* p = sub.c_str();
    for (size_t i = 0; i < count; ++i) {
        while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r' || *p == ',') ++p;
        char* endPtr = nullptr;
        const double v = std::strtod(p, &endPtr);
        if (endPtr == p || !std::isfinite(v)) return false;
        outArr[i] = v;
        p = endPtr;
    }
    return true;
}

Vec3 sampleRawGyroCamAt(
    const std::vector<GFGyroRecord>& gyro,
    const double M[9],
    double t
) {
    if (gyro.empty()) return Vec3(0.0, 0.0, 0.0);
    auto toCam = [&](const GFGyroRecord& g) -> Vec3 {
        return Vec3(
            M[0] * g.gx + M[1] * g.gy + M[2] * g.gz,
            M[3] * g.gx + M[4] * g.gy + M[5] * g.gz,
            M[6] * g.gx + M[7] * g.gy + M[8] * g.gz
        );
    };
    if (t <= gyro.front().timestamp_sec) return toCam(gyro.front());
    if (t >= gyro.back().timestamp_sec) return toCam(gyro.back());

    auto it = std::lower_bound(
        gyro.begin(), gyro.end(), t,
        [](const GFGyroRecord& rec, double v) { return rec.timestamp_sec < v; }
    );
    if (it == gyro.begin()) return toCam(*it);
    const auto& g1 = *it;
    const auto& g0 = *(it - 1);
    const double span = g1.timestamp_sec - g0.timestamp_sec;
    if (span <= 1e-9) return toCam(g0);
    const double alpha = (t - g0.timestamp_sec) / span;
    const Vec3 c0 = toCam(g0);
    const Vec3 c1 = toCam(g1);
    return c0 + (c1 - c0) * alpha;
}

/// Exact trapezoidal average of raw camera gyro over [t0, t1] (avoids finite-difference sinc bias).
Vec3 integrateAverageRawGyroCam(
    const std::vector<GFGyroRecord>& gyro,
    const double M[9],
    double t0,
    double t1
) {
    if (gyro.empty() || t1 <= t0 + 1e-7) {
        return sampleRawGyroCamAt(gyro, M, 0.5 * (t0 + t1));
    }

    auto it = std::upper_bound(
        gyro.begin(), gyro.end(), t0,
        [](double val, const GFGyroRecord& rec) { return val < rec.timestamp_sec; }
    );

    double prevT = t0;
    Vec3 prevW = sampleRawGyroCamAt(gyro, M, t0);
    Vec3 integral(0.0, 0.0, 0.0);

    auto toCam = [&](const GFGyroRecord& g) -> Vec3 {
        return Vec3(
            M[0] * g.gx + M[1] * g.gy + M[2] * g.gz,
            M[3] * g.gx + M[4] * g.gy + M[5] * g.gz,
            M[6] * g.gx + M[7] * g.gy + M[8] * g.gz
        );
    };

    while (it != gyro.end() && it->timestamp_sec < t1) {
        const double curT = it->timestamp_sec;
        const Vec3 curW = toCam(*it);
        const double h = curT - prevT;
        if (h > 0.0) {
            integral = integral + (prevW + curW) * (0.5 * h);
        }
        prevT = curT;
        prevW = curW;
        ++it;
    }

    const Vec3 endW = sampleRawGyroCamAt(gyro, M, t1);
    const double hEnd = t1 - prevT;
    if (hEnd > 0.0) {
        integral = integral + (prevW + endW) * (0.5 * hEnd);
    }
    return integral * (1.0 / (t1 - t0));
}

double solveHuberLocation1D(const std::vector<double>& vals, const std::vector<double>& baseWeights, double huberDelta) {
    if (vals.empty()) return 0.0;
    std::vector<double> sorted = vals;
    std::sort(sorted.begin(), sorted.end());
    double mu = sorted[sorted.size() / 2];

    for (int iter = 0; iter < 15; ++iter) {
        double sumW = 0.0;
        double sumWY = 0.0;
        for (size_t i = 0; i < vals.size(); ++i) {
            const double r = std::fabs(vals[i] - mu);
            const double hW = (r <= huberDelta) ? 1.0 : (huberDelta / (r + 1e-9));
            const double w = baseWeights[i] * hW;
            sumW += w;
            sumWY += w * vals[i];
        }
        if (sumW > 1e-12) {
            mu = sumWY / sumW;
        }
    }
    return mu;
}

void solveHuberDeming2D(
    const std::vector<double>& v,
    const std::vector<double>& g,
    const std::vector<double>& baseW,
    double deltaVarianceRatio,
    double huberDelta,
    double slopeInit,
    double interceptInit,
    double& outSlope,
    double& outIntercept
) {
    outSlope = slopeInit;
    outIntercept = interceptInit;
    const size_t n = v.size();
    if (n < 2) return;

    for (int iter = 0; iter < 18; ++iter) {
        double sumW = 0.0, meanV = 0.0, meanG = 0.0;
        std::vector<double> weights(n, 0.0);
        for (size_t i = 0; i < n; ++i) {
            const double res = g[i] - (outSlope * v[i] + outIntercept);
            const double r = std::fabs(res);
            const double hW = (r <= huberDelta) ? 1.0 : (huberDelta / (r + 1e-9));
            const double w = baseW[i] * hW;
            weights[i] = w;
            sumW += w;
            meanV += w * v[i];
            meanG += w * g[i];
        }
        if (sumW <= 1e-12) break;
        meanV /= sumW;
        meanG /= sumW;

        double svv = 0.0, sgg = 0.0, svg = 0.0;
        for (size_t i = 0; i < n; ++i) {
            const double dv = v[i] - meanV;
            const double dg = g[i] - meanG;
            svv += weights[i] * dv * dv;
            sgg += weights[i] * dg * dg;
            svg += weights[i] * dv * dg;
        }
        if (std::fabs(svg) < 1e-12 || svv < 1e-12) break;

        const double D = sgg - deltaVarianceRatio * svv;
        const double disc = std::sqrt(D * D + 4.0 * deltaVarianceRatio * svg * svg);
        // Numerically stable root with same sign as svg (minimizes orthogonal Deming loss)
        const double nextSlope = (D >= 0.0)
            ? ((D + disc) / (2.0 * svg))
            : ((2.0 * deltaVarianceRatio * svg) / (disc - D));
        if (!std::isfinite(nextSlope)) break;
        const double nextIntercept = meanG - nextSlope * meanV;
        const double diff = std::fabs(nextSlope - outSlope) + std::fabs(nextIntercept - outIntercept);
        outSlope = nextSlope;
        outIntercept = nextIntercept;
        if (diff < 1e-8) break;
    }
}

Quat integrateCalibratedSegment(
    const std::vector<GFGyroRecord>& sortedGyro,
    const double M[9],
    const double biasCam[3],
    const double scaleCam[3],
    double offsetA,
    double offsetB,
    double tRef,
    double tCam0,
    double tCam1
) {
    if (std::fabs(tCam1 - tCam0) < 1e-7) return Quat::identity();
    const double sign = (tCam1 >= tCam0) ? 1.0 : -1.0;
    const double tStart = std::min(tCam0, tCam1);
    const double tEnd = std::max(tCam0, tCam1);
    const int subSteps = std::max(2, static_cast<int>(std::ceil((tEnd - tStart) / 0.004)));
    const double dtSub = (tEnd - tStart) / static_cast<double>(subSteps);

    Quat q = Quat::identity();
    for (int s = 0; s < subSteps; ++s) {
        const double tMidCam = tStart + (static_cast<double>(s) + 0.5) * dtSub;
        const double tGyro = tMidCam + (offsetA + offsetB * (tMidCam - tRef)) * 0.001;
        const Vec3 wRaw = sampleRawGyroCamAt(sortedGyro, M, tGyro);
        const Vec3 wCal(
            (wRaw.x - biasCam[0]) / scaleCam[0],
            (wRaw.y - biasCam[1]) / scaleCam[1],
            (wRaw.z - biasCam[2]) / scaleCam[2]
        );
        q = Quat::integrateGyroStep(q, wCal * sign, dtSub);
    }
    return q;
}

} // namespace

std::string RefinedCalibration::toJson() const {
    char buf[1024];
    std::snprintf(
        buf, sizeof(buf),
        "{\n"
        "  \"valid\": %s,\n"
        "  \"usedPriorFallback\": %s,\n"
        "  \"axisPresetIndex\": %d,\n"
        "  \"offsetA\": %.6f,\n"
        "  \"offsetB\": %.6f,\n"
        "  \"referenceStartSec\": %.6f,\n"
        "  \"scale\": [%.6f, %.6f, %.6f],\n"
        "  \"bias\": [%.6f, %.6f, %.6f],\n"
        "  \"readoutMs\": %.6f,\n"
        "  \"focalScale\": %.6f,\n"
        "  \"confidence\": %.6f,\n"
        "  \"residualRmsBefore\": %.6f,\n"
        "  \"residualRmsAfter\": %.6f,\n"
        "  \"axisMatrix\": [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n"
        "}",
        valid ? "true" : "false",
        usedPriorFallback ? "true" : "false",
        axisPresetIndex,
        offsetA,
        offsetB,
        referenceStartSec,
        scale[0], scale[1], scale[2],
        bias[0], bias[1], bias[2],
        readoutMs,
        focalScale,
        confidence,
        residualRmsBefore,
        residualRmsAfter,
        axisMatrix[0], axisMatrix[1], axisMatrix[2],
        axisMatrix[3], axisMatrix[4], axisMatrix[5],
        axisMatrix[6], axisMatrix[7], axisMatrix[8]
    );
    return std::string(buf);
}

bool RefinedCalibration::fromJson(const std::string& jsonText, RefinedCalibration& outCal) {
    RefinedCalibration cal;
    double presetIdxD = 0.0;
    if (!extractJsonBool(jsonText, "valid", cal.valid)) return false;
    extractJsonBool(jsonText, "usedPriorFallback", cal.usedPriorFallback);
    if (extractJsonNumber(jsonText, "axisPresetIndex", presetIdxD)) {
        cal.axisPresetIndex = static_cast<int>(std::llround(presetIdxD));
    }
    extractJsonNumber(jsonText, "offsetA", cal.offsetA);
    extractJsonNumber(jsonText, "offsetB", cal.offsetB);
    extractJsonNumber(jsonText, "referenceStartSec", cal.referenceStartSec);
    extractJsonArray(jsonText, "scale", cal.scale, 3);
    extractJsonArray(jsonText, "bias", cal.bias, 3);
    extractJsonNumber(jsonText, "readoutMs", cal.readoutMs);
    extractJsonNumber(jsonText, "focalScale", cal.focalScale);
    extractJsonNumber(jsonText, "confidence", cal.confidence);
    extractJsonNumber(jsonText, "residualRmsBefore", cal.residualRmsBefore);
    extractJsonNumber(jsonText, "residualRmsAfter", cal.residualRmsAfter);
    extractJsonArray(jsonText, "axisMatrix", cal.axisMatrix, 9);
    outCal = cal;
    return true;
}

bool RefinedCalibration::saveJsonFile(const std::string& path, const RefinedCalibration& cal) {
    std::ofstream ofs(path);
    if (!ofs) return false;
    ofs << cal.toJson() << "\n";
    return static_cast<bool>(ofs);
}

bool RefinedCalibration::loadJsonFile(const std::string& path, RefinedCalibration& outCal) {
    std::ifstream ifs(path);
    if (!ifs) return false;
    std::ostringstream oss;
    oss << ifs.rdbuf();
    return fromJson(oss.str(), outCal);
}

CalibrationRefiner::CalibrationRefiner() = default;

double CalibrationRefiner::computeMedian(std::vector<double> values) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const size_t mid = values.size() / 2;
    if (values.size() % 2 == 1) return values[mid];
    return 0.5 * (values[mid - 1] + values[mid]);
}

CalibrationModePrior CalibrationRefiner::computeModePriorFromHistory(
    const std::vector<double>& readoutMsHist,
    const std::vector<double>& focalScaleHist,
    const std::vector<double>& scaleXHist,
    const std::vector<double>& scaleYHist,
    const std::vector<double>& scaleZHist,
    double defaultReadoutMs
) {
    CalibrationModePrior prior;
    prior.readoutMs = defaultReadoutMs;
    if (!readoutMsHist.empty()) {
        prior.readoutMs = computeMedian(readoutMsHist);
        prior.hasPrior = true;
    }
    if (!focalScaleHist.empty()) {
        prior.focalScale = std::max(0.85, std::min(1.20, computeMedian(focalScaleHist)));
        prior.hasPrior = true;
    }
    if (!scaleXHist.empty() && !scaleYHist.empty() && !scaleZHist.empty()) {
        prior.scale[0] = std::max(0.85, std::min(1.15, computeMedian(scaleXHist)));
        prior.scale[1] = std::max(0.85, std::min(1.15, computeMedian(scaleYHist)));
        prior.scale[2] = std::max(0.85, std::min(1.15, computeMedian(scaleZHist)));
        prior.hasPrior = true;
    }
    return prior;
}

void CalibrationRefiner::reset(
    const GFFileHeader& header,
    const std::vector<GFGyroRecord>& gyroRecords,
    const std::vector<GFFrameRecord>& frames,
    const CalibrationModePrior* prior,
    const OpticalSyncResult* initialSync,
    bool motoActionMode
) {
    header_ = header;
    sortedGyro_ = gyroRecords;
    std::sort(sortedGyro_.begin(), sortedGyro_.end(), [](const GFGyroRecord& a, const GFGyroRecord& b) {
        return a.timestamp_sec < b.timestamp_sec;
    });
    frames_ = frames;
    prior_ = prior ? *prior : CalibrationModePrior{};
    if (!prior || !prior->hasPrior) {
        const double hdrRsMs = (header_.readout_time_sec > 1e-5) ? (header_.readout_time_sec * 1000.0) : 15.2;
        prior_.readoutMs = hdrRsMs;
        prior_.focalScale = 1.0;
        prior_.scale[0] = 1.0;
        prior_.scale[1] = 1.0;
        prior_.scale[2] = 1.0;
    }
    if (initialSync && initialSync->valid) {
        initialSync_ = *initialSync;
        hasInitialSync_ = true;
    } else {
        initialSync_ = OpticalSyncResult{};
        hasInitialSync_ = false;
    }
    motoActionMode_ = motoActionMode;
    firstFrameW_ = 0;
    firstFrameH_ = 0;
    hasPrev_ = false;
    prevCenterSec_ = 0.0;
    prevPyr_ = VirtualGyroEngine::FramePyramid{};
    currPyr_ = VirtualGyroEngine::FramePyramid{};
    visPairs_.clear();
    pointFrames_.clear();
}

bool CalibrationRefiner::pushDownsampledLumaFrame(
    const uint8_t* luma,
    int width,
    int height,
    double ptsSec,
    size_t frameIndex
) {
    if (!luma || width < 32 || height < 32) {
        return false;
    }
    if (firstFrameW_ == 0) {
        firstFrameW_ = width;
        firstFrameH_ = height;
    }
    if (width != firstFrameW_ || height != firstFrameH_) {
        return false;
    }

    double tCenter = ptsSec;
    if (frameIndex < frames_.size()) {
        tCenter = frames_[frameIndex].pts_sec
            + 0.5 * static_cast<double>(frames_[frameIndex].exposure_duration_sec)
            + 0.5 * static_cast<double>(frames_[frameIndex].rolling_shutter_sec);
    } else {
        tCenter = ptsSec + 0.5 * std::max(0.0, header_.readout_time_sec);
    }

    if (!hasPrev_) {
        VirtualGyroEngine::buildPyramid(luma, width, height, tCenter, prevPyr_);
        prevCenterSec_ = tCenter;
        hasPrev_ = true;
        return true;
    }

    VirtualGyroEngine::buildPyramid(luma, width, height, tCenter, currPyr_);

    const double baseW = (header_.width > 0) ? static_cast<double>(header_.width) : 1920.0;
    const double baseH = (header_.height > 0) ? static_cast<double>(header_.height) : 1080.0;
    const double baseLong = std::max(baseW, baseH);
    double rawFx = (header_.lens_fx > 10.0) ? header_.lens_fx : (1815.0 * (baseLong / 1920.0));
    double rawFy = (header_.lens_fy > 10.0) ? header_.lens_fy : rawFx;
    if (baseLong >= 1900.0 && (rawFx < 1500.0 || std::fabs(rawFx - 1660.0) < 2.0)) {
        rawFx = 1815.0;
        rawFy = 1815.0;
    } else if (baseLong >= 1200.0 && baseLong < 1500.0 && (rawFx < 1000.0 || std::fabs(rawFx - 1106.67) < 2.0)) {
        rawFx = 1210.0;
        rawFy = 1210.0;
    }
    const double initFocalScale = prior_.focalScale;

    VirtualGyroConfig vgCfg;
    vgCfg.videoWidth = static_cast<uint32_t>(baseW);
    vgCfg.videoHeight = static_cast<uint32_t>(baseH);
    vgCfg.nominalFps = (header_.nominal_fps > 1.0) ? header_.nominal_fps : 30.0;
    vgCfg.readoutTimeSec = prior_.readoutMs * 0.001;
    vgCfg.lensFx = rawFx * initFocalScale;
    vgCfg.lensFy = rawFy * initFocalScale;
    vgCfg.lensCx = (header_.lens_cx > 10.0) ? header_.lens_cx : (0.5 * baseW);
    vgCfg.lensCy = (header_.lens_cy > 10.0) ? header_.lens_cy : (0.5 * baseH);

    VirtualGyroEngine::VisualOmegaSample sample{};
    double inlierRatio = 0.0;
    int inlierCount = 0;
    std::vector<VirtualGyroEngine::TrackedPointPair> trackedPts;
    const bool ok = VirtualGyroEngine::estimateRotationBetweenPyramids(
        prevPyr_, currPyr_, vgCfg, sample, inlierRatio, &inlierCount, &trackedPts
    );

    VisPairSample vp{};
    vp.t0 = prevCenterSec_;
    vp.t1 = tCenter;
    vp.tMid = 0.5 * (prevCenterSec_ + tCenter);
    vp.dt = tCenter - prevCenterSec_;
    if (ok && vp.dt > 1e-4 && vp.dt <= 0.25) {
        vp.wxVis = sample.wxCam;
        vp.wyVis = sample.wyCam;
        vp.wzVis = sample.wzCam;
        vp.inlierRatio = inlierRatio;
        vp.inlierCount = inlierCount;
        vp.valid = true;

        if (!trackedPts.empty()) {
            const bool scanAlongX = (baseW < baseH);
            const double spanDim = scanAlongX ? std::max(1.0, baseW - 1.0) : std::max(1.0, baseH - 1.0);
            double sumR0 = 0.0, sumR1 = 0.0;
            for (const auto& pt : trackedPts) {
                sumR0 += ((scanAlongX ? pt.u0 : pt.v0) / spanDim) - 0.5;
                sumR1 += ((scanAlongX ? pt.u1 : pt.v1) / spanDim) - 0.5;
            }
            vp.meanRow0 = sumR0 / static_cast<double>(trackedPts.size());
            vp.meanRow1 = sumR1 / static_cast<double>(trackedPts.size());
        }

        // Keep up to 16 evenly spaced inliers for Step 6 RS/focal point-flow refinement (bounded RAM <= 200 frames)
        const double omegaNorm = std::sqrt(vp.wxVis * vp.wxVis + vp.wyVis * vp.wyVis + vp.wzVis * vp.wzVis);
        if (inlierRatio >= 0.45 && inlierCount >= 8 && omegaNorm > 0.12 && !trackedPts.empty()) {
            PointPairFrame pf;
            pf.t0 = vp.t0;
            pf.t1 = vp.t1;
            pf.motionScore = omegaNorm * inlierRatio;
            const size_t stride = std::max<size_t>(1, trackedPts.size() / 16);
            for (size_t k = 0; k < trackedPts.size() && pf.pts.size() < 16; k += stride) {
                pf.pts.push_back(trackedPts[k]);
            }
            if (pointFrames_.size() < 200) {
                pointFrames_.push_back(std::move(pf));
            } else {
                auto minIt = std::min_element(
                    pointFrames_.begin(), pointFrames_.end(),
                    [](const PointPairFrame& a, const PointPairFrame& b) { return a.motionScore < b.motionScore; }
                );
                if (minIt != pointFrames_.end() && pf.motionScore > minIt->motionScore) {
                    *minIt = std::move(pf);
                }
            }
        }
    }
    visPairs_.push_back(vp);

    std::swap(prevPyr_, currPyr_);
    prevCenterSec_ = tCenter;
    return true;
}

RefinedCalibration CalibrationRefiner::finalize() {
    RefinedCalibration out;
    const auto& presets = OpticalSync::getAxisPresets();
    out.axisPresetIndex = (hasInitialSync_ && initialSync_.bestAxisPresetIndex >= 0 &&
                           static_cast<size_t>(initialSync_.bestAxisPresetIndex) < presets.size())
        ? initialSync_.bestAxisPresetIndex
        : 0;
    std::copy(std::begin(presets[static_cast<size_t>(out.axisPresetIndex)].m),
              std::end(presets[static_cast<size_t>(out.axisPresetIndex)].m),
              std::begin(out.axisMatrix));
    if (hasInitialSync_) {
        std::copy(std::begin(initialSync_.axisMatrix), std::end(initialSync_.axisMatrix), std::begin(out.axisMatrix));
    }

    const double headerBiasCamArr[3] = {
        out.axisMatrix[0] * header_.gyro_bias_x + out.axisMatrix[1] * header_.gyro_bias_y + out.axisMatrix[2] * header_.gyro_bias_z,
        out.axisMatrix[3] * header_.gyro_bias_x + out.axisMatrix[4] * header_.gyro_bias_y + out.axisMatrix[5] * header_.gyro_bias_z,
        out.axisMatrix[6] * header_.gyro_bias_x + out.axisMatrix[7] * header_.gyro_bias_y + out.axisMatrix[8] * header_.gyro_bias_z
    };

    out.bias[0] = headerBiasCamArr[0];
    out.bias[1] = headerBiasCamArr[1];
    out.bias[2] = headerBiasCamArr[2];
    out.scale[0] = prior_.scale[0];
    out.scale[1] = prior_.scale[1];
    out.scale[2] = prior_.scale[2];
    out.readoutMs = (hasInitialSync_ && initialSync_.hasReadoutEstimate) ? initialSync_.signedReadoutTimeMs : prior_.readoutMs;
    out.focalScale = prior_.focalScale;
    out.offsetA = hasInitialSync_ ? initialSync_.bestOffsetMs : 0.0;
    out.offsetB = 0.0;
    out.referenceStartSec = visPairs_.empty() ? 0.0 : visPairs_.front().t0;

    if (visPairs_.size() < 8 || sortedGyro_.size() < 20) {
        out.usedPriorFallback = true;
        out.valid = true;
        return out;
    }

    // Collect valid pairs for Step 1 axis/coarse init if needed and Step 3-7 refinement
    std::vector<size_t> goodIdx;
    goodIdx.reserve(visPairs_.size());
    double inlierSum = 0.0;
    double visEnergyTotal = 0.0;
    for (size_t i = 0; i < visPairs_.size(); ++i) {
        const auto& vp = visPairs_[i];
        if (vp.valid && vp.inlierRatio >= 0.45 && vp.inlierCount >= 8) {
            goodIdx.push_back(i);
            inlierSum += vp.inlierRatio;
            visEnergyTotal += (vp.wxVis * vp.wxVis + vp.wyVis * vp.wyVis + vp.wzVis * vp.wzVis) * vp.dt;
        }
    }

    const double validFrac = static_cast<double>(goodIdx.size()) / static_cast<double>(visPairs_.size());
    const double meanInlier = goodIdx.empty() ? 0.0 : (inlierSum / static_cast<double>(goodIdx.size()));

    // Step 1 fallback/verification: if initialSync wasn't provided, solve axis preset + coarse [-80,+80]ms offset on streamed pairs
    if (!hasInitialSync_ && goodIdx.size() >= 8 && visEnergyTotal >= 0.005) {
        int bestP = 0;
        double bestOff = 0.0;
        double bestScore = -1.0;
        for (size_t pIdx = 0; pIdx < presets.size(); ++pIdx) {
            const double* M = presets[pIdx].m;
            for (int off = -80; off <= 80; off += 2) {
                const double offSec = off * 0.001;
                double dotSum = 0.0, vSq = 0.0, gSq = 0.0;
                Vec3 meanG(0, 0, 0), meanV(0, 0, 0);
                std::vector<Vec3> gList;
                gList.reserve(goodIdx.size());
                for (size_t idx : goodIdx) {
                    const auto& vp = visPairs_[idx];
                    const Vec3 gAvg = integrateAverageRawGyroCam(sortedGyro_, M, vp.t0 + offSec, vp.t1 + offSec);
                    gList.push_back(gAvg);
                    meanG = meanG + gAvg;
                    meanV = meanV + Vec3(vp.wxVis, vp.wyVis, vp.wzVis);
                }
                const double invN = 1.0 / static_cast<double>(goodIdx.size());
                meanG = meanG * invN;
                meanV = meanV * invN;
                for (size_t k = 0; k < goodIdx.size(); ++k) {
                    const auto& vp = visPairs_[goodIdx[k]];
                    const Vec3 dv = Vec3(vp.wxVis, vp.wyVis, vp.wzVis) - meanV;
                    const Vec3 dg = gList[k] - meanG;
                    dotSum += dv.dot(dg);
                    vSq += dv.normSquared();
                    gSq += dg.normSquared();
                }
                const double den = std::sqrt(vSq * gSq);
                const double ncc = (den > 1e-9) ? (dotSum / den) : 0.0;
                if (ncc > bestScore) {
                    bestScore = ncc;
                    bestP = static_cast<int>(pIdx);
                    bestOff = static_cast<double>(off);
                }
            }
        }
        out.axisPresetIndex = bestP;
        std::copy(std::begin(presets[static_cast<size_t>(bestP)].m),
                  std::end(presets[static_cast<size_t>(bestP)].m),
                  std::begin(out.axisMatrix));
        out.offsetA = bestOff;
        out.bias[0] = out.axisMatrix[0] * header_.gyro_bias_x + out.axisMatrix[1] * header_.gyro_bias_y + out.axisMatrix[2] * header_.gyro_bias_z;
        out.bias[1] = out.axisMatrix[3] * header_.gyro_bias_x + out.axisMatrix[4] * header_.gyro_bias_y + out.axisMatrix[5] * header_.gyro_bias_z;
        out.bias[2] = out.axisMatrix[6] * header_.gyro_bias_x + out.axisMatrix[7] * header_.gyro_bias_y + out.axisMatrix[8] * header_.gyro_bias_z;
    }

    const double* M = out.axisMatrix;
    const double initOffsetMs = out.offsetA;

    // Helper to compute angular velocity residual RMS over goodIdx (rolling-shutter aware)
    auto computeResidualRms = [&](double offA, double offB, double rsMs, const double sc[3], const double bi[3]) -> double {
        if (goodIdx.empty()) return 0.0;
        const double rsSec = rsMs * 0.001;
        double sumSq = 0.0;
        for (size_t idx : goodIdx) {
            const auto& vp = visPairs_[idx];
            const double tv0 = vp.t0 + vp.meanRow0 * rsSec;
            const double tv1 = vp.t1 + vp.meanRow1 * rsSec;
            const double dtEff = std::max(1e-4, tv1 - tv0);
            const double d0 = (offA + offB * (tv0 - out.referenceStartSec)) * 0.001;
            const double d1 = (offA + offB * (tv1 - out.referenceStartSec)) * 0.001;
            const Vec3 gAvg = integrateAverageRawGyroCam(sortedGyro_, M, tv0 + d0, tv1 + d1);
            const double scaleDt = vp.dt / dtEff;
            const double ex = gAvg.x - (sc[0] * (vp.wxVis * scaleDt) + bi[0]);
            const double ey = gAvg.y - (sc[1] * (vp.wyVis * scaleDt) + bi[1]);
            const double ez = gAvg.z - (sc[2] * (vp.wzVis * scaleDt) + bi[2]);
            sumSq += ex * ex + ey * ey + ez * ez;
        }
        return std::sqrt(sumSq / static_cast<double>(3 * goodIdx.size()));
    };

    const double unitScale[3] = {1.0, 1.0, 1.0};
    out.residualRmsBefore = computeResidualRms(initOffsetMs, 0.0, out.readoutMs, unitScale, out.bias);

    // Low-texture or low-motion early gate -> fallback to prior cleanly
    if (goodIdx.size() < 10 || validFrac < 0.35 || meanInlier < 0.50 || visEnergyTotal < 0.008) {
        out.confidence = std::min(0.40, validFrac * meanInlier);
        out.residualRmsAfter = out.residualRmsBefore;
        out.usedPriorFallback = true;
        out.valid = true;
        return out;
    }

    // Step 3: Sliding-window offset estimation (4s window, 2s step; sweep +-20ms @ 1ms + parabolic peak)
    const double tClipStart = visPairs_[goodIdx.front()].t0;
    const double tClipEnd = visPairs_[goodIdx.back()].t1;
    const double clipSpan = std::max(0.0, tClipEnd - tClipStart);
    const double winLen = (clipSpan >= 4.0) ? 4.0 : std::max(1.0, clipSpan);
    const double winStep = 2.0;

    struct WinOffset {
        double tMid;
        double offsetMs;
        double weight;
        double peakNcc;
    };
    std::vector<WinOffset> winOffsets;

    for (double wStart = tClipStart; wStart <= tClipEnd - 0.5 * winLen + 1e-4; wStart += winStep) {
        const double wEnd = wStart + winLen;
        std::vector<size_t> winPairs;
        double winEnergy = 0.0;
        for (size_t idx : goodIdx) {
            const auto& vp = visPairs_[idx];
            if (vp.tMid >= wStart && vp.tMid <= wEnd) {
                winPairs.push_back(idx);
                winEnergy += (vp.wxVis * vp.wxVis + vp.wyVis * vp.wyVis + vp.wzVis * vp.wzVis) * vp.dt;
            }
        }
        if (winPairs.size() < 8 || winEnergy < 0.005) {
            continue;
        }

        auto evalWindowNcc = [&](double offMs) -> double {
            const double offSec = offMs * 0.001;
            std::vector<Vec3> gArr;
            gArr.reserve(winPairs.size());
            Vec3 meanG(0, 0, 0), meanV(0, 0, 0);
            for (size_t idx : winPairs) {
                const auto& vp = visPairs_[idx];
                const Vec3 gAvg = integrateAverageRawGyroCam(sortedGyro_, M, vp.t0 + offSec, vp.t1 + offSec);
                gArr.push_back(gAvg);
                meanG = meanG + gAvg;
                meanV = meanV + Vec3(vp.wxVis, vp.wyVis, vp.wzVis);
            }
            const double invN = 1.0 / static_cast<double>(winPairs.size());
            meanG = meanG * invN;
            meanV = meanV * invN;

            double dotSum = 0.0, vSq = 0.0, gSq = 0.0;
            double dDot = 0.0, dvSq = 0.0, dgSq = 0.0;
            for (size_t k = 0; k < winPairs.size(); ++k) {
                const auto& vp = visPairs_[winPairs[k]];
                const Vec3 v = Vec3(vp.wxVis, vp.wyVis, vp.wzVis) - meanV;
                const Vec3 g = gArr[k] - meanG;
                dotSum += v.dot(g);
                vSq += v.normSquared();
                gSq += g.normSquared();
                if (k > 0) {
                    const auto& vpPrev = visPairs_[winPairs[k - 1]];
                    const Vec3 dv = Vec3(vp.wxVis - vpPrev.wxVis, vp.wyVis - vpPrev.wyVis, vp.wzVis - vpPrev.wzVis);
                    const Vec3 dg = gArr[k] - gArr[k - 1];
                    dDot += dv.dot(dg);
                    dvSq += dv.normSquared();
                    dgSq += dg.normSquared();
                }
            }
            const double vDen = std::sqrt(vSq * gSq);
            const double velNcc = (vDen > 1e-9) ? (dotSum / vDen) : 0.0;
            const double dDen = std::sqrt(dvSq * dgSq);
            const double accNcc = (dDen > 1e-7) ? (dDot / dDen) : velNcc;
            return 0.70 * velNcc + 0.30 * accNcc;
        };

        // Sweep +-20 ms around initOffsetMs at 1 ms step
        std::array<double, 41> curve{};
        int bestK = 20;
        double bestVal = -2.0;
        for (int k = 0; k <= 40; ++k) {
            const double candMs = initOffsetMs + static_cast<double>(k - 20);
            const double val = evalWindowNcc(candMs);
            curve[static_cast<size_t>(k)] = val;
            if (val > bestVal) {
                bestVal = val;
                bestK = k;
            }
        }

        // Sub-ms parabolic interpolation around peak + tiny golden polish
        double peakOffsetMs = initOffsetMs + static_cast<double>(bestK - 20);
        if (bestK > 0 && bestK < 40) {
            const double yL = curve[static_cast<size_t>(bestK - 1)];
            const double yC = curve[static_cast<size_t>(bestK)];
            const double yR = curve[static_cast<size_t>(bestK + 1)];
            const double denom = 2.0 * (yL - 2.0 * yC + yR);
            if (std::fabs(denom) > 1e-8 && denom < 0.0) {
                const double shift = std::min(0.5, std::max(-0.5, (yL - yR) / denom));
                peakOffsetMs += shift;
            }
        }
        const double refinedNcc = evalWindowNcc(peakOffsetMs);
        if (refinedNcc >= 0.65) {
            const double tWinMid = 0.5 * (visPairs_[winPairs.front()].tMid + visPairs_[winPairs.back()].tMid);
            winOffsets.push_back({tWinMid, peakOffsetMs, refinedNcc * std::sqrt(winEnergy), refinedNcc});
        }
        if (wEnd >= tClipEnd - 1e-4) break;
    }

    // Step 4: Fit linear drift delta(t) = a + b1 * (t - referenceStartSec) via Huber IRLS
    double solvedOffsetA = initOffsetMs;
    double solvedOffsetB = 0.0;
    double avgPeakNcc = hasInitialSync_ ? initialSync_.bestNCC : 0.75;
    if (winOffsets.size() >= 2) {
        std::vector<double> wt, wy, ww;
        double nccSum = 0.0;
        double tMinW = 1e18, tMaxW = -1e18;
        for (const auto& wo : winOffsets) {
            wt.push_back(wo.tMid - out.referenceStartSec);
            wy.push_back(wo.offsetMs);
            ww.push_back(wo.weight);
            nccSum += wo.peakNcc;
            tMinW = std::min(tMinW, wo.tMid);
            tMaxW = std::max(tMaxW, wo.tMid);
        }
        avgPeakNcc = nccSum / static_cast<double>(winOffsets.size());
        if (tMaxW - tMinW >= 1.5) {
            // Time variable t is exact (sigma_t ~ 0), so delta = sigma_y^2 / sigma_t^2 -> large (ordinary Huber regression)
            solveHuberDeming2D(wt, wy, ww, 1e6, 1.0, 0.0, initOffsetMs, solvedOffsetB, solvedOffsetA);
            solvedOffsetB = std::max(-5.0, std::min(5.0, solvedOffsetB));
        } else {
            solvedOffsetA = solveHuberLocation1D(wy, ww, 1.0);
        }
    } else if (winOffsets.size() == 1) {
        solvedOffsetA = winOffsets[0].offsetMs;
        solvedOffsetB = 0.0;
        avgPeakNcc = winOffsets[0].peakNcc;
    }

    // Step 5: Hold delta(t) fixed, solve per-axis s_i, b_i via Huber IRLS (accounting for RS row-shift interval)
    auto solvePerAxisScaleAndBias = [&](double focalVisScaleFactor, double rsMs, double outScale[3], double outBias[3]) {
        const size_t nGood = goodIdx.size();
        const double rsSec = rsMs * 0.001;
        std::vector<double> gX(nGood), gY(nGood), gZ(nGood);
        std::vector<double> vX(nGood), vY(nGood), vZ(nGood), pairW(nGood);

        for (size_t k = 0; k < nGood; ++k) {
            const auto& vp = visPairs_[goodIdx[k]];
            const double tv0 = vp.t0 + vp.meanRow0 * rsSec;
            const double tv1 = vp.t1 + vp.meanRow1 * rsSec;
            const double dtEff = std::max(1e-4, tv1 - tv0);
            const double d0 = (solvedOffsetA + solvedOffsetB * (tv0 - out.referenceStartSec)) * 0.001;
            const double d1 = (solvedOffsetA + solvedOffsetB * (tv1 - out.referenceStartSec)) * 0.001;
            const Vec3 gAvg = integrateAverageRawGyroCam(sortedGyro_, M, tv0 + d0, tv1 + d1);
            gX[k] = gAvg.x; gY[k] = gAvg.y; gZ[k] = gAvg.z;
            const double scaleDt = vp.dt / dtEff;
            // When focalScale updates from initFocalScale -> new, wx/wy scale inversely with focal
            vX[k] = (vp.wxVis * scaleDt) / focalVisScaleFactor;
            vY[k] = (vp.wyVis * scaleDt) / focalVisScaleFactor;
            vZ[k] = (vp.wzVis * scaleDt);
            pairW[k] = vp.inlierRatio * std::min(1.0, static_cast<double>(vp.inlierCount) / 16.0);
        }

        auto solveAxis = [&](const std::vector<double>& vArr, const std::vector<double>& gArr, int axisIdx) {
            double meanV = 0.0;
            for (double v : vArr) meanV += v;
            meanV /= static_cast<double>(nGood);
            double varV = 0.0;
            for (double v : vArr) {
                const double d = v - meanV;
                varV += d * d;
            }
            const double stdV = std::sqrt(varV / static_cast<double>(nGood));

            if (stdV >= 0.05 && nGood >= 10) {
                double s = prior_.scale[axisIdx];
                double b = out.bias[axisIdx];
                // Gyro noise sigma_g << visual flow noise sigma_v (delta = sigma_g^2 / sigma_v^2 = 0.04) -> removes attenuation bias
                solveHuberDeming2D(vArr, gArr, pairW, 0.04, 0.012, s, b, s, b);
                outScale[axisIdx] = std::max(0.85, std::min(1.15, s));
                // Re-estimate bias robustly if scale got clamped
                std::vector<double> resB(nGood);
                for (size_t k = 0; k < nGood; ++k) resB[k] = gArr[k] - outScale[axisIdx] * vArr[k];
                outBias[axisIdx] = solveHuberLocation1D(resB, pairW, 0.010);
            } else {
                outScale[axisIdx] = prior_.scale[axisIdx];
                std::vector<double> resB(nGood);
                for (size_t k = 0; k < nGood; ++k) resB[k] = gArr[k] - outScale[axisIdx] * vArr[k];
                outBias[axisIdx] = solveHuberLocation1D(resB, pairW, 0.010);
            }
        };

        solveAxis(vX, gX, 0);
        solveAxis(vY, gY, 1);
        solveAxis(vZ, gZ, 2);
    };

    double curScale[3] = {prior_.scale[0], prior_.scale[1], prior_.scale[2]};
    double curBias[3] = {out.bias[0], out.bias[1], out.bias[2]};
    solvePerAxisScaleAndBias(1.0, out.readoutMs, curScale, curBias);

    // Step 6: Refine signedReadoutTimeMs and effectiveFocalScale via RS point-flow coordinate descent (no image render!)
    const double baseW = (header_.width > 0) ? static_cast<double>(header_.width) : 1920.0;
    const double baseH = (header_.height > 0) ? static_cast<double>(header_.height) : 1080.0;
    const double baseLong = std::max(baseW, baseH);
    double rawFx = (header_.lens_fx > 10.0) ? header_.lens_fx : (1815.0 * (baseLong / 1920.0));
    double rawFy = (header_.lens_fy > 10.0) ? header_.lens_fy : rawFx;
    if (baseLong >= 1900.0 && (rawFx < 1500.0 || std::fabs(rawFx - 1660.0) < 2.0)) {
        rawFx = 1815.0; rawFy = 1815.0;
    } else if (baseLong >= 1200.0 && baseLong < 1500.0 && (rawFx < 1000.0 || std::fabs(rawFx - 1106.67) < 2.0)) {
        rawFx = 1210.0; rawFy = 1210.0;
    }
    const double cxFull = (header_.lens_cx > 10.0) ? header_.lens_cx : (0.5 * baseW);
    const double cyFull = (header_.lens_cy > 10.0) ? header_.lens_cy : (0.5 * baseH);
    const double initFocalScale = prior_.focalScale;
    const bool scanAlongX = (baseW < baseH);

    double curReadoutMs = out.readoutMs;
    double curFocalScale = initFocalScale;

    if (pointFrames_.size() >= 6) {
        auto evalFlowCost = [&](double rsMs, double fScale, const double sc[3], const double bi[3]) -> double {
            const double fx = rawFx * fScale;
            const double fy = rawFy * fScale;
            const double rsSec = rsMs * 0.001;
            double totalCost = 0.0;
            size_t totalPts = 0;

            for (const auto& pf : pointFrames_) {
                for (const auto& pt : pf.pts) {
                    const double r0 = scanAlongX
                        ? ((pt.u0 / std::max(1.0, baseW - 1.0)) - 0.5)
                        : ((pt.v0 / std::max(1.0, baseH - 1.0)) - 0.5);
                    const double r1 = scanAlongX
                        ? ((pt.u1 / std::max(1.0, baseW - 1.0)) - 0.5)
                        : ((pt.v1 / std::max(1.0, baseH - 1.0)) - 0.5);
                    const double tp0 = pf.t0 + r0 * rsSec;
                    const double tp1 = pf.t1 + r1 * rsSec;

                    const Quat dq = integrateCalibratedSegment(
                        sortedGyro_, M, bi, sc, solvedOffsetA, solvedOffsetB, out.referenceStartSec, tp0, tp1
                    );
                    const Vec3 ray0((pt.u0 - cxFull) / fx, (pt.v0 - cyFull) / fy, 1.0);
                    const Vec3 ray1 = dq.conjugate().rotateVector(ray0);
                    if (ray1.z <= 0.1) continue;
                    const double predU1 = fx * (ray1.x / ray1.z) + cxFull;
                    const double predV1 = fy * (ray1.y / ray1.z) + cyFull;
                    const double errSq = (predU1 - pt.u1) * (predU1 - pt.u1) + (predV1 - pt.v1) * (predV1 - pt.v1);
                    const double huberCap = 36.0; // (6 px)^2 cap in full-res space
                    totalCost += (errSq <= huberCap) ? errSq : (2.0 * std::sqrt(huberCap * errSq) - huberCap);
                    ++totalPts;
                }
            }
            if (totalPts == 0) return 1e18;
            // Gentle anchor toward prior.focalScale so focalScale stays decoupled from anisotropic gyro scales
            const double focalPriorPenalty = 12.0 * (fScale - prior_.focalScale) * (fScale - prior_.focalScale);
            return (totalCost / static_cast<double>(totalPts)) + focalPriorPenalty;
        };

        // 6A: Coarse + golden-section search on readoutMs in [-25, +25] ms
        double bestRs = curReadoutMs;
        double bestCost = evalFlowCost(bestRs, curFocalScale, curScale, curBias);
        for (int rs = -24; rs <= 24; rs += 2) {
            const double c = evalFlowCost(static_cast<double>(rs), curFocalScale, curScale, curBias);
            if (c < bestCost) {
                bestCost = c;
                bestRs = static_cast<double>(rs);
            }
        }
        double gsA = std::max(-25.0, bestRs - 2.0);
        double gsB = std::min(25.0, bestRs + 2.0);
        constexpr double kInvPhi = 0.6180339887498949;
        double gsC = gsB - kInvPhi * (gsB - gsA);
        double gsD = gsA + kInvPhi * (gsB - gsA);
        double fC = evalFlowCost(gsC, curFocalScale, curScale, curBias);
        double fD = evalFlowCost(gsD, curFocalScale, curScale, curBias);
        for (int iter = 0; iter < 12; ++iter) {
            if (fC < fD) {
                gsB = gsD; gsD = gsC; fD = fC;
                gsC = gsB - kInvPhi * (gsB - gsA);
                fC = evalFlowCost(gsC, curFocalScale, curScale, curBias);
            } else {
                gsA = gsC; gsC = gsD; fC = fD;
                gsD = gsA + kInvPhi * (gsB - gsA);
                fD = evalFlowCost(gsD, curFocalScale, curScale, curBias);
            }
        }
        const double candRs = 0.5 * (gsA + gsB);
        const double candCost = evalFlowCost(candRs, curFocalScale, curScale, curBias);
        if (candCost < bestCost) {
            curReadoutMs = candRs;
            bestCost = candCost;
        }
        solvePerAxisScaleAndBias(curFocalScale / initFocalScale, curReadoutMs, curScale, curBias);

        // 6B: Coordinate check on focalScale around prior (only move if pixel residual genuinely drops)
        const double fStep[5] = {-0.03, -0.015, 0.0, 0.015, 0.03};
        double bestF = curFocalScale;
        double baseFCost = evalFlowCost(curReadoutMs, curFocalScale, curScale, curBias);
        for (double df : fStep) {
            const double testF = std::max(0.88, std::min(1.15, curFocalScale + df));
            double testScale[3], testBias[3];
            solvePerAxisScaleAndBias(testF / initFocalScale, curReadoutMs, testScale, testBias);
            const double c = evalFlowCost(curReadoutMs, testF, testScale, testBias);
            if (c < baseFCost * 0.985) {
                baseFCost = c;
                bestF = testF;
            }
        }
        if (std::fabs(bestF - curFocalScale) > 1e-4) {
            curFocalScale = bestF;
            solvePerAxisScaleAndBias(curFocalScale / initFocalScale, curReadoutMs, curScale, curBias);
        }
    }

    const double rmsAfter = computeResidualRms(solvedOffsetA, solvedOffsetB, curReadoutMs, curScale, curBias);

    // Step 7: Overall confidence & fallback check
    const double nccScore = std::min(1.0, std::max(0.0, (avgPeakNcc - 0.30) / 0.65));
    const double fracScore = std::min(1.0, validFrac / 0.65);
    const double inlierScore = std::min(1.0, std::max(0.0, (meanInlier - 0.40) / 0.45));
    const double confidence = std::min(1.0, std::max(0.0, 0.45 * nccScore + 0.30 * inlierScore + 0.25 * fracScore));

    out.confidence = confidence;
    if (confidence < 0.42 || (out.residualRmsBefore > 1e-4 && rmsAfter > out.residualRmsBefore * 1.02)) {
        out.residualRmsAfter = out.residualRmsBefore;
        out.usedPriorFallback = true;
        out.valid = true;
        return out;
    }

    out.offsetA = solvedOffsetA;
    out.offsetB = solvedOffsetB;
    out.scale[0] = curScale[0];
    out.scale[1] = curScale[1];
    out.scale[2] = curScale[2];
    out.bias[0] = curBias[0];
    out.bias[1] = curBias[1];
    out.bias[2] = curBias[2];
    out.readoutMs = curReadoutMs;
    out.focalScale = curFocalScale;
    out.residualRmsAfter = rmsAfter;
    out.usedPriorFallback = false;
    out.valid = true;
    return out;
}

} // namespace gf
