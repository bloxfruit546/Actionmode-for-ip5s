#include "OpticalSync.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

namespace gf {

namespace {

int computeSadPatch(
    const uint8_t* img0, int w0,
    const uint8_t* img1, int w1,
    int u0, int v0,
    int u1, int v1,
    int radius
) {
    int sad = 0;
    for (int dy = -radius; dy <= radius; ++dy) {
        const uint8_t* r0 = img0 + (v0 + dy) * w0 + (u0 - radius);
        const uint8_t* r1 = img1 + (v1 + dy) * w1 + (u1 - radius);
        for (int dx = 0; dx <= 2 * radius; ++dx) {
            sad += std::abs(static_cast<int>(r0[dx]) - static_cast<int>(r1[dx]));
        }
    }
    return sad;
}

std::vector<uint8_t> downsample2x(const std::vector<uint8_t>& src, int w, int h, int& outW, int& outH) {
    outW = w / 2;
    outH = h / 2;
    std::vector<uint8_t> dst(static_cast<size_t>(outW * outH), 0);
    for (int y = 0; y < outH; ++y) {
        const uint8_t* r0 = src.data() + (2 * y) * w;
        const uint8_t* r1 = src.data() + (2 * y + 1) * w;
        uint8_t* d = dst.data() + y * outW;
        for (int x = 0; x < outW; ++x) {
            const int sum = r0[2 * x] + r0[2 * x + 1] + r1[2 * x] + r1[2 * x + 1];
            d[x] = static_cast<uint8_t>(sum >> 2);
        }
    }
    return dst;
}

bool solve3x3(double A[3][3], double b[3], double x[3]) {
    // Gaussian elimination with partial pivoting
    for (int col = 0; col < 3; ++col) {
        int pivot = col;
        double maxVal = std::fabs(A[col][col]);
        for (int r = col + 1; r < 3; ++r) {
            if (std::fabs(A[r][col]) > maxVal) {
                maxVal = std::fabs(A[r][col]);
                pivot = r;
            }
        }
        if (maxVal < 1e-10) {
            return false;
        }
        if (pivot != col) {
            for (int c = col; c < 3; ++c) {
                std::swap(A[col][c], A[pivot][c]);
            }
            std::swap(b[col], b[pivot]);
        }
        const double invDiag = 1.0 / A[col][col];
        for (int r = col + 1; r < 3; ++r) {
            const double factor = A[r][col] * invDiag;
            for (int c = col; c < 3; ++c) {
                A[r][c] -= factor * A[col][c];
            }
            b[r] -= factor * b[col];
        }
    }

    for (int r = 2; r >= 0; --r) {
        double sum = b[r];
        for (int c = r + 1; c < 3; ++c) {
            sum -= A[r][c] * x[c];
        }
        x[r] = sum / A[r][r];
    }
    return true;
}

struct SampledGyro {
    double gx, gy, gz;
};

SampledGyro sampleGyroAt(const std::vector<GFGyroRecord>& gyro, double t) {
    if (gyro.empty()) {
        return {0.0, 0.0, 0.0};
    }
    if (t <= gyro.front().timestamp_sec) {
        return {gyro.front().gx, gyro.front().gy, gyro.front().gz};
    }
    if (t >= gyro.back().timestamp_sec) {
        return {gyro.back().gx, gyro.back().gy, gyro.back().gz};
    }
    auto it = std::lower_bound(
        gyro.begin(), gyro.end(), t,
        [](const GFGyroRecord& rec, double val) { return rec.timestamp_sec < val; }
    );
    if (it == gyro.begin()) {
        return {it->gx, it->gy, it->gz};
    }
    const auto& g1 = *it;
    const auto& g0 = *(it - 1);
    const double span = g1.timestamp_sec - g0.timestamp_sec;
    if (span <= 1e-9) {
        return {g0.gx, g0.gy, g0.gz};
    }
    const double alpha = (t - g0.timestamp_sec) / span;
    return {
        g0.gx + alpha * (g1.gx - g0.gx),
        g0.gy + alpha * (g1.gy - g0.gy),
        g0.gz + alpha * (g1.gz - g0.gz)
    };
}

} // namespace

const std::vector<AxisPreset>& OpticalSync::getAxisPresets() {
    static const std::vector<AxisPreset> presets = {
        {"LandscapeLeft (-y,-x,-z)",        { 0, -1,  0, -1,  0,  0,  0,  0, -1}},
        {"LandscapeRight (+y,+x,-z)",       { 0,  1,  0,  1,  0,  0,  0,  0, -1}},
        {"Portrait (+x,-y,-z)",             { 1,  0,  0,  0, -1,  0,  0,  0, -1}},
        {"PortraitUpsideDown (-x,+y,-z)",   {-1,  0,  0,  0,  1,  0,  0,  0, -1}},
        {"LandscapeLeft_Alt (-y,+x,+z)",    { 0, -1,  0,  1,  0,  0,  0,  0,  1}},
        {"LandscapeRight_Alt (+y,-x,+z)",   { 0,  1,  0, -1,  0,  0,  0,  0,  1}},
        {"Portrait_Alt (+x,+y,+z)",         { 1,  0,  0,  0,  1,  0,  0,  0,  1}},
        {"PortraitUpside_Alt (-x,-y,+z)",   {-1,  0,  0,  0, -1,  0,  0,  0,  1}}
    };
    return presets;
}

bool OpticalSync::estimateVisualOmega(
    const OpticalFlowFrame& f0,
    const OpticalFlowFrame& f1,
    double fx,
    double fy,
    double cx,
    double cy,
    double& outWx,
    double& outWy,
    double& outWz
) {
    outWx = 0.0;
    outWy = 0.0;
    outWz = 0.0;

    const int w = f0.width;
    const int h = f0.height;
    const double dt = f1.timestampSec - f0.timestampSec;
    if (w < 64 || h < 64 || f1.width != w || f1.height != h ||
        f0.luma.size() != static_cast<size_t>(w * h) ||
        f1.luma.size() != static_cast<size_t>(w * h) ||
        dt <= 1e-4 || dt > 0.25) {
        return false;
    }

    int hw = 0, hh = 0;
    const std::vector<uint8_t> h0 = downsample2x(f0.luma, w, h, hw, hh);
    const std::vector<uint8_t> h1 = downsample2x(f1.luma, w, h, hw, hh);

    const int margin = 20;
    const int gridCols = 6;
    const int gridRows = 4;
    const int cellW = (w - 2 * margin) / gridCols;
    const int cellH = (h - 2 * margin) / gridRows;

    struct FlowObs {
        double x, y, dx, dy;
        double weight;
        int gy;
    };
    std::vector<FlowObs> obs;
    obs.reserve(gridCols * gridRows);

    for (int gy = 0; gy < gridRows; ++gy) {
        for (int gx = 0; gx < gridCols; ++gx) {
            const int xStart = margin + gx * cellW;
            const int yStart = margin + gy * cellH;

            int bestU = xStart + cellW / 2;
            int bestV = yStart + cellH / 2;
            int bestScore = -1;

            for (int v = yStart + 2; v < yStart + cellH - 2; v += 2) {
                for (int u = xStart + 2; u < xStart + cellW - 2; u += 2) {
                    const int ix = static_cast<int>(f0.luma[v * w + u + 1]) - static_cast<int>(f0.luma[v * w + u - 1]);
                    const int iy = static_cast<int>(f0.luma[(v + 1) * w + u]) - static_cast<int>(f0.luma[(v - 1) * w + u]);
                    const int score = std::min(std::abs(ix), std::abs(iy));
                    if (score > bestScore) {
                        bestScore = score;
                        bestU = u;
                        bestV = v;
                    }
                }
            }

            if (bestScore < 6) {
                continue;
            }

            // Coarse search on 2x downsampled image (radius 10 -> +-20px full res)
            const int hu0 = bestU / 2;
            const int hv0 = bestV / 2;
            const int cRad = 3;
            const int cSearch = 10;
            if (hu0 - cRad - cSearch < 1 || hu0 + cRad + cSearch >= hw - 1 ||
                hv0 - cRad - cSearch < 1 || hv0 + cRad + cSearch >= hh - 1) {
                continue;
            }

            int bestHu1 = hu0;
            int bestHv1 = hv0;
            int bestCSad = 1 << 30;
            for (int dv = -cSearch; dv <= cSearch; ++dv) {
                for (int du = -cSearch; du <= cSearch; ++du) {
                    const int sad = computeSadPatch(h0.data(), hw, h1.data(), hw, hu0, hv0, hu0 + du, hv0 + dv, cRad);
                    if (sad < bestCSad) {
                        bestCSad = sad;
                        bestHu1 = hu0 + du;
                        bestHv1 = hv0 + dv;
                    }
                }
            }

            // Fine search on full-res image around coarse match
            const int fCenterU = bestHu1 * 2;
            const int fCenterV = bestHv1 * 2;
            const int fRad = 4;
            const int fSearch = 3;
            if (bestU - fRad < 1 || bestU + fRad >= w - 1 ||
                bestV - fRad < 1 || bestV + fRad >= h - 1 ||
                fCenterU - fRad - fSearch < 1 || fCenterU + fRad + fSearch >= w - 1 ||
                fCenterV - fRad - fSearch < 1 || fCenterV + fRad + fSearch >= h - 1) {
                continue;
            }

            int bestFu1 = fCenterU;
            int bestFv1 = fCenterV;
            int bestFSad = 1 << 30;
            for (int dv = -fSearch; dv <= fSearch; ++dv) {
                for (int du = -fSearch; du <= fSearch; ++du) {
                    const int sad = computeSadPatch(f0.luma.data(), w, f1.luma.data(), w, bestU, bestV, fCenterU + du, fCenterV + dv, fRad);
                    if (sad < bestFSad) {
                        bestFSad = sad;
                        bestFu1 = fCenterU + du;
                        bestFv1 = fCenterV + dv;
                    }
                }
            }

            // Sub-pixel parabolic refinement in X and Y
            double subU = static_cast<double>(bestFu1);
            double subV = static_cast<double>(bestFv1);
            if (bestFu1 - fRad - 1 >= 0 && bestFu1 + fRad + 1 < w) {
                const int sL = computeSadPatch(f0.luma.data(), w, f1.luma.data(), w, bestU, bestV, bestFu1 - 1, bestFv1, fRad);
                const int sC = bestFSad;
                const int sR = computeSadPatch(f0.luma.data(), w, f1.luma.data(), w, bestU, bestV, bestFu1 + 1, bestFv1, fRad);
                const double denom = 2.0 * (sL - 2.0 * sC + sR);
                if (std::fabs(denom) > 1e-6) {
                    subU += std::min(0.5, std::max(-0.5, (sL - sR) / denom));
                }
            }
            if (bestFv1 - fRad - 1 >= 0 && bestFv1 + fRad + 1 < h) {
                const int sU = computeSadPatch(f0.luma.data(), w, f1.luma.data(), w, bestU, bestV, bestFu1, bestFv1 - 1, fRad);
                const int sC = bestFSad;
                const int sD = computeSadPatch(f0.luma.data(), w, f1.luma.data(), w, bestU, bestV, bestFu1, bestFv1 + 1, fRad);
                const double denom = 2.0 * (sU - 2.0 * sC + sD);
                if (std::fabs(denom) > 1e-6) {
                    subV += std::min(0.5, std::max(-0.5, (sU - sD) / denom));
                }
            }

            const double du = subU - static_cast<double>(bestU);
            const double dv = subV - static_cast<double>(bestV);

            obs.push_back({
                (static_cast<double>(bestU) - cx) / fx,
                (static_cast<double>(bestV) - cy) / fy,
                du / fx,
                dv / fy,
                1.0,
                gy
            });
        }
    }

    if (obs.size() < 6) {
        return false;
    }

    // 2-iteration IRLS to reject moving-object outliers
    double omegaDt[3] = {0.0, 0.0, 0.0};
    for (int iter = 0; iter < 2; ++iter) {
        double ATA[3][3] = {};
        double ATb[3] = {};

        for (const auto& o : obs) {
            const double rowX[3] = {o.x * o.y, -(1.0 + o.x * o.x), o.y};
            const double rowY[3] = {(1.0 + o.y * o.y), -o.x * o.y, -o.x};

            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    ATA[r][c] += o.weight * (rowX[r] * rowX[c] + rowY[r] * rowY[c]);
                }
                ATb[r] += o.weight * (rowX[r] * o.dx + rowY[r] * o.dy);
            }
        }

        if (!solve3x3(ATA, ATb, omegaDt)) {
            return false;
        }

        for (auto& o : obs) {
            const double predDx = omegaDt[0] * (o.x * o.y) - omegaDt[1] * (1.0 + o.x * o.x) + omegaDt[2] * o.y;
            const double predDy = omegaDt[0] * (1.0 + o.y * o.y) - omegaDt[1] * (o.x * o.y) - omegaDt[2] * o.x;
            const double err = std::hypot(o.dx - predDx, o.dy - predDy);
            o.weight = (err < 0.004) ? 1.0 : (0.004 / (err + 1e-6));
        }
    }

    outWx = omegaDt[0] / dt;
    outWy = omegaDt[1] / dt;
    outWz = omegaDt[2] / dt;
    return true;
}

OpticalSyncResult OpticalSync::solveSyncAndAxes(
    const std::vector<OpticalFlowFrame>& frames,
    const GFFileHeader& header,
    const std::vector<GFGyroRecord>& gyroRecords
) {
    OpticalSyncResult result;
    const auto& presets = getAxisPresets();
    result.axisPresetName = presets[0].name;
    std::copy(std::begin(presets[0].m), std::end(presets[0].m), std::begin(result.axisMatrix));

    if (frames.size() < 10 || gyroRecords.size() < 20) {
        result.diagnosticMessage = "Insufficient frames or gyro samples for Auto-Sync.";
        return result;
    }

    std::vector<GFGyroRecord> sortedGyro = gyroRecords;
    std::sort(sortedGyro.begin(), sortedGyro.end(), [](const GFGyroRecord& a, const GFGyroRecord& b) {
        return a.timestamp_sec < b.timestamp_sec;
    });

    const double baseW = (header.width > 0) ? static_cast<double>(header.width) : 1920.0;
    const double scale = static_cast<double>(frames[0].width) / baseW;
    const double fx = ((header.lens_fx > 10.0) ? header.lens_fx : 1660.0) * scale;
    const double fy = ((header.lens_fy > 10.0) ? header.lens_fy : 1660.0) * scale;
    const double cx = ((header.lens_cx > 10.0) ? header.lens_cx : 960.0) * scale;
    const double cy = ((header.lens_cy > 10.0) ? header.lens_cy : 540.0) * scale;

    struct VisSample {
        double tMid;
        double wx, wy, wz;
    };
    std::vector<VisSample> visSamples;
    visSamples.reserve(frames.size());

    double visEnergy = 0.0;
    for (size_t i = 1; i < frames.size(); ++i) {
        double wx = 0.0, wy = 0.0, wz = 0.0;
        if (estimateVisualOmega(frames[i - 1], frames[i], fx, fy, cx, cy, wx, wy, wz)) {
            const double tMid = 0.5 * (frames[i - 1].timestampSec + frames[i].timestampSec);
            visSamples.push_back({tMid, wx, wy, wz});
            visEnergy += wx * wx + wy * wy + wz * wz;
        }
    }

    if (visSamples.size() < 8 || visEnergy < 1e-3) {
        result.valid = true;
        result.bestOffsetMs = 0.0;
        result.bestNCC = 0.0;
        result.diagnosticMessage = "Static scene (low visual motion); using default axis & 0.0 ms.";
        return result;
    }

    const int numOffsets = 81; // -80 ms .. +80 ms step 2 ms
    int bestPreset = 0;
    int bestOffIdx = 40;
    double bestNccOverall = -1.0;
    std::vector<double> bestPresetCurve(numOffsets, 0.0);

    for (size_t pIdx = 0; pIdx < presets.size(); ++pIdx) {
        const double* M = presets[pIdx].m;
        std::vector<double> curve(numOffsets, 0.0);

        for (int oi = 0; oi < numOffsets; ++oi) {
            const double offSec = (-80.0 + oi * 2.0) * 0.001;
            double dotSum = 0.0;
            double visNormSq = 0.0;
            double imuNormSq = 0.0;

            for (const auto& vs : visSamples) {
                const SampledGyro raw = sampleGyroAt(sortedGyro, vs.tMid + offSec);
                const double gx = raw.gx - header.gyro_bias_x;
                const double gy = raw.gy - header.gyro_bias_y;
                const double gz = raw.gz - header.gyro_bias_z;

                const double ix = M[0] * gx + M[1] * gy + M[2] * gz;
                const double iy = M[3] * gx + M[4] * gy + M[5] * gz;
                const double iz = M[6] * gx + M[7] * gy + M[8] * gz;

                dotSum += vs.wx * ix + vs.wy * iy + vs.wz * iz;
                visNormSq += vs.wx * vs.wx + vs.wy * vs.wy + vs.wz * vs.wz;
                imuNormSq += ix * ix + iy * iy + iz * iz;
            }

            const double denom = std::sqrt(visNormSq * imuNormSq);
            const double ncc = (denom > 1e-9) ? (dotSum / denom) : 0.0;
            curve[oi] = ncc;

            if (ncc > bestNccOverall) {
                bestNccOverall = ncc;
                bestPreset = static_cast<int>(pIdx);
                bestOffIdx = oi;
                bestPresetCurve = curve;
            }
        }
    }

    auto evalNccAtOffsetMs = [&](double offMs) -> double {
        const double* M = presets[bestPreset].m;
        const double offSec = offMs * 0.001;
        double dotSum = 0.0;
        double visNormSq = 0.0;
        double imuNormSq = 0.0;
        for (const auto& vs : visSamples) {
            const SampledGyro raw = sampleGyroAt(sortedGyro, vs.tMid + offSec);
            const double gx = raw.gx - header.gyro_bias_x;
            const double gy = raw.gy - header.gyro_bias_y;
            const double gz = raw.gz - header.gyro_bias_z;
            const double ix = M[0] * gx + M[1] * gy + M[2] * gz;
            const double iy = M[3] * gx + M[4] * gy + M[5] * gz;
            const double iz = M[6] * gx + M[7] * gy + M[8] * gz;
            dotSum += vs.wx * ix + vs.wy * iy + vs.wz * iz;
            visNormSq += vs.wx * vs.wx + vs.wy * vs.wy + vs.wz * vs.wz;
            imuNormSq += ix * ix + iy * iy + iz * iz;
        }
        const double denom = std::sqrt(visNormSq * imuNormSq);
        return (denom > 1e-9) ? (dotSum / denom) : 0.0;
    };

    // Golden-Section Search to refine offset to < 0.1 ms precision
    const double coarseCenterMs = -80.0 + bestOffIdx * 2.0;
    double gsA = std::max(-80.0, coarseCenterMs - 2.0);
    double gsB = std::min(80.0, coarseCenterMs + 2.0);
    constexpr double kInvPhi = 0.6180339887498949;
    double gsC = gsB - kInvPhi * (gsB - gsA);
    double gsD = gsA + kInvPhi * (gsB - gsA);
    double fC = evalNccAtOffsetMs(gsC);
    double fD = evalNccAtOffsetMs(gsD);
    for (int iter = 0; iter < 14; ++iter) {
        if (fC < fD) {
            gsA = gsC;
            gsC = gsD;
            fC = fD;
            gsD = gsA + kInvPhi * (gsB - gsA);
            fD = evalNccAtOffsetMs(gsD);
        } else {
            gsB = gsD;
            gsD = gsC;
            fD = fC;
            gsC = gsB - kInvPhi * (gsB - gsA);
            fC = evalNccAtOffsetMs(gsC);
        }
    }
    const double refinedOffsetMs = 0.5 * (gsA + gsB);
    const double refinedNcc = evalNccAtOffsetMs(refinedOffsetMs);
    if (refinedNcc > bestNccOverall) {
        bestNccOverall = refinedNcc;
    }

    result.valid = true;
    result.bestOffsetMs = refinedOffsetMs;
    result.bestNCC = bestNccOverall;
    result.bestAxisPresetIndex = bestPreset;
    result.axisPresetName = presets[bestPreset].name;
    std::copy(std::begin(presets[bestPreset].m), std::end(presets[bestPreset].m), std::begin(result.axisMatrix));

    // IRLS effective focal scale estimation (compensates focus breathing / lens tolerance)
    result.effectiveFocalScale = 1.0;
    result.hasFocalScaleEstimate = false;
    if (bestNccOverall >= 0.70 && visSamples.size() >= 8) {
        const double* M = presets[bestPreset].m;
        const double offSec = refinedOffsetMs * 0.001;
        struct FocalPair {
            double vwx, vwy, iwx, iwy, weight;
        };
        std::vector<FocalPair> pairs;
        pairs.reserve(visSamples.size());
        double imuXYEnergy = 0.0;
        for (const auto& vs : visSamples) {
            const SampledGyro raw = sampleGyroAt(sortedGyro, vs.tMid + offSec);
            const double gx = raw.gx - header.gyro_bias_x;
            const double gy = raw.gy - header.gyro_bias_y;
            const double gz = raw.gz - header.gyro_bias_z;
            const double ix = M[0] * gx + M[1] * gy + M[2] * gz;
            const double iy = M[3] * gx + M[4] * gy + M[5] * gz;
            const double magSq = ix * ix + iy * iy;
            if (magSq > 1e-4) {
                pairs.push_back({vs.wx, vs.wy, ix, iy, 1.0});
                imuXYEnergy += magSq;
            }
        }
        if (pairs.size() >= 6 && imuXYEnergy > 0.02) {
            double focalScale = 1.0;
            for (int iter = 0; iter < 3; ++iter) {
                double num = 0.0;
                double den = 0.0;
                for (const auto& p : pairs) {
                    num += p.weight * (p.vwx * p.iwx + p.vwy * p.iwy);
                    den += p.weight * (p.iwx * p.iwx + p.iwy * p.iwy);
                }
                if (den <= 1e-9) break;
                focalScale = num / den;
                for (auto& p : pairs) {
                    const double ex = p.vwx - focalScale * p.iwx;
                    const double ey = p.vwy - focalScale * p.iwy;
                    const double err = std::hypot(ex, ey);
                    p.weight = (err < 0.05) ? 1.0 : (0.05 / (err + 1e-6));
                }
            }
            if (std::isfinite(focalScale)) {
                result.effectiveFocalScale = std::max(0.85, std::min(1.20, focalScale));
                result.hasFocalScaleEstimate = true;
            }
        }
    }

    // Estimate signed rolling-shutter readout time (top-half vs bottom-half row lag)
    const double defaultRsMs = (header.readout_time_sec > 1e-5) ? (header.readout_time_sec * 1000.0) : 15.2;
    result.signedReadoutTimeMs = defaultRsMs;
    result.hasReadoutEstimate = false;

    if (bestNccOverall >= 0.60 && frames.size() >= 15) {
        const double* M = presets[bestPreset].m;
        const double offSec = refinedOffsetMs * 0.001;

        // Helper to estimate visual omega restricted to top (gy < 2) or bottom (gy >= 2) half
        struct HalfVis {
            double tMid;
            double yFracTop, yFracBot;
            double wxTop, wyTop, wzTop;
            double wxBot, wyBot, wzBot;
        };
        std::vector<HalfVis> halfSamples;
        for (size_t i = 1; i < frames.size(); ++i) {
            const auto& f0 = frames[i - 1];
            const auto& f1 = frames[i];
            const int w = f0.width;
            const int h = f0.height;
            const double dt = f1.timestampSec - f0.timestampSec;
            if (w < 64 || h < 64 || dt <= 1e-4 || dt > 0.25) continue;

            auto solveHalf = [&](int gyStart, int gyEnd, double& outYFrac, double& owx, double& owy, double& owz) -> bool {
                const int margin = 20;
                const int gridCols = 6;
                const int gridRows = 4;
                const int cellW = (w - 2 * margin) / gridCols;
                const int cellH = (h - 2 * margin) / gridRows;
                struct HFlow { double x, y, dx, dy; };
                std::vector<HFlow> hobs;
                double yFracSum = 0.0;
                for (int gy = gyStart; gy < gyEnd; ++gy) {
                    for (int gx = 0; gx < gridCols; ++gx) {
                        const int xStart = margin + gx * cellW;
                        const int yStart = margin + gy * cellH;
                        int bestU = xStart + cellW / 2, bestV = yStart + cellH / 2, bestScore = -1;
                        for (int v = yStart + 2; v < yStart + cellH - 2; v += 2) {
                            for (int u = xStart + 2; u < xStart + cellW - 2; u += 2) {
                                const int ix = static_cast<int>(f0.luma[v * w + u + 1]) - static_cast<int>(f0.luma[v * w + u - 1]);
                                const int iy = static_cast<int>(f0.luma[(v + 1) * w + u]) - static_cast<int>(f0.luma[(v - 1) * w + u]);
                                const int sc = std::min(std::abs(ix), std::abs(iy));
                                if (sc > bestScore) { bestScore = sc; bestU = u; bestV = v; }
                            }
                        }
                        if (bestScore < 6) continue;
                        const int fRad = 4, fSearch = 18;
                        if (bestU - fRad < 1 || bestU + fRad >= w - 1 ||
                            bestV - fRad < 1 || bestV + fRad >= h - 1) continue;
                        const int uMin = std::max(1 + fRad, bestU - fSearch);
                        const int uMax = std::min(w - 2 - fRad, bestU + fSearch);
                        const int vMin = std::max(1 + fRad, bestV - fSearch);
                        const int vMax = std::min(h - 2 - fRad, bestV + fSearch);
                        int bU = bestU, bV = bestV, bSad = 1 << 30;
                        for (int tv = vMin; tv <= vMax; ++tv) {
                            for (int tu = uMin; tu <= uMax; ++tu) {
                                const int sad = computeSadPatch(f0.luma.data(), w, f1.luma.data(), w, bestU, bestV, tu, tv, fRad);
                                if (sad < bSad) { bSad = sad; bU = tu; bV = tv; }
                            }
                        }
                        double subU = bU, subV = bV;
                        if (bU - fRad - 1 >= 0 && bU + fRad + 1 < w) {
                            const int sL = computeSadPatch(f0.luma.data(), w, f1.luma.data(), w, bestU, bestV, bU - 1, bV, fRad);
                            const int sR = computeSadPatch(f0.luma.data(), w, f1.luma.data(), w, bestU, bestV, bU + 1, bV, fRad);
                            const double den = 2.0 * (sL - 2.0 * bSad + sR);
                            if (std::fabs(den) > 1e-6) subU += std::min(0.5, std::max(-0.5, (sL - sR) / den));
                        }
                        if (bV - fRad - 1 >= 0 && bV + fRad + 1 < h) {
                            const int sU = computeSadPatch(f0.luma.data(), w, f1.luma.data(), w, bestU, bestV, bU, bV - 1, fRad);
                            const int sD = computeSadPatch(f0.luma.data(), w, f1.luma.data(), w, bestU, bestV, bU, bV + 1, fRad);
                            const double den = 2.0 * (sU - 2.0 * bSad + sD);
                            if (std::fabs(den) > 1e-6) subV += std::min(0.5, std::max(-0.5, (sU - sD) / den));
                        }
                        hobs.push_back({(bestU - cx) / fx, (bestV - cy) / fy, (subU - bestU) / fx, (subV - bestV) / fy});
                        yFracSum += (static_cast<double>(bestV) / static_cast<double>(h - 1)) - 0.5;
                    }
                }
                if (hobs.size() < 4) return false;
                outYFrac = yFracSum / static_cast<double>(hobs.size());
                double ATA[3][3] = {}, ATb[3] = {}, x[3] = {};
                for (const auto& o : hobs) {
                    const double rx[3] = {o.x * o.y, -(1.0 + o.x * o.x), o.y};
                    const double ry[3] = {(1.0 + o.y * o.y), -o.x * o.y, -o.x};
                    for (int r = 0; r < 3; ++r) {
                        for (int c = 0; c < 3; ++c) ATA[r][c] += rx[r] * rx[c] + ry[r] * ry[c];
                        ATb[r] += rx[r] * o.dx + ry[r] * o.dy;
                    }
                }
                if (!solve3x3(ATA, ATb, x)) return false;
                owx = x[0] / dt; owy = x[1] / dt; owz = x[2] / dt;
                return true;
            };

            HalfVis hv{};
            hv.tMid = 0.5 * (f0.timestampSec + f1.timestampSec);
            if (solveHalf(0, 2, hv.yFracTop, hv.wxTop, hv.wyTop, hv.wzTop) &&
                solveHalf(2, 4, hv.yFracBot, hv.wxBot, hv.wyBot, hv.wzBot)) {
                halfSamples.push_back(hv);
            }
        }

        if (halfSamples.size() >= 6) {
            double bestErr = 1e18;
            double worstErr = 0.0;
            double bestRsMs = defaultRsMs;
            for (int rs = -20; rs <= 20; ++rs) {
                const double rsSec = rs * 0.001;
                double errSum = 0.0;
                for (const auto& hv : halfSamples) {
                    const auto gTop = sampleGyroAt(sortedGyro, hv.tMid + offSec + hv.yFracTop * rsSec);
                    const double tgx = gTop.gx - header.gyro_bias_x, tgy = gTop.gy - header.gyro_bias_y, tgz = gTop.gz - header.gyro_bias_z;
                    const double itx = M[0] * tgx + M[1] * tgy + M[2] * tgz;
                    const double ity = M[3] * tgx + M[4] * tgy + M[5] * tgz;
                    const double itz = M[6] * tgx + M[7] * tgy + M[8] * tgz;

                    const auto gBot = sampleGyroAt(sortedGyro, hv.tMid + offSec + hv.yFracBot * rsSec);
                    const double bgx = gBot.gx - header.gyro_bias_x, bgy = gBot.gy - header.gyro_bias_y, bgz = gBot.gz - header.gyro_bias_z;
                    const double ibx = M[0] * bgx + M[1] * bgy + M[2] * bgz;
                    const double iby = M[3] * bgx + M[4] * bgy + M[5] * bgz;
                    const double ibz = M[6] * bgx + M[7] * bgy + M[8] * bgz;

                    const double dTop = (hv.wxTop - itx) * (hv.wxTop - itx) + (hv.wyTop - ity) * (hv.wyTop - ity) + (hv.wzTop - itz) * (hv.wzTop - itz);
                    const double dBot = (hv.wxBot - ibx) * (hv.wxBot - ibx) + (hv.wyBot - iby) * (hv.wyBot - iby) + (hv.wzBot - ibz) * (hv.wzBot - ibz);
                    errSum += dTop + dBot;
                }
                if (errSum < bestErr) {
                    bestErr = errSum;
                    bestRsMs = static_cast<double>(rs);
                }
                if (errSum > worstErr) {
                    worstErr = errSum;
                }
            }
            if (worstErr > bestErr * 1.03) {
                result.signedReadoutTimeMs = bestRsMs;
                result.hasReadoutEstimate = true;
            }
        }
    }

    char buf[240];
    std::snprintf(
        buf, sizeof(buf),
        "Auto-Sync: %+.1f ms | NCC: %.3f | RS: %+.1f ms | Scale: %.3fx | Axis: %s",
        result.bestOffsetMs, result.bestNCC, result.signedReadoutTimeMs,
        result.effectiveFocalScale, result.axisPresetName.c_str()
    );
    result.diagnosticMessage = buf;
    return result;
}

} // namespace gf
