#include "OpticalSync.hpp"
#include "FrameWarpCalculator.hpp"

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
    double& outWz,
    bool motoActionMode
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

    const int xMin = motoActionMode ? std::max(20, static_cast<int>(0.15 * w)) : 20;
    const int xMax = motoActionMode ? std::min(w - 20, static_cast<int>(0.85 * w)) : (w - 20);
    const int yMin = motoActionMode ? std::max(20, static_cast<int>(0.10 * h)) : 20;
    const int yMax = motoActionMode ? std::min(h - 20, static_cast<int>(0.48 * h)) : (h - 20);
    const int gridCols = 6;
    const int gridRows = 4;
    const int cellW = std::max(8, (xMax - xMin) / gridCols);
    const int cellH = std::max(8, (yMax - yMin) / gridRows);

    struct FlowObs {
        double x, y, dx, dy;
        double weight;
        int gy;
    };
    std::vector<FlowObs> obs;
    obs.reserve(gridCols * gridRows);

    for (int gy = 0; gy < gridRows; ++gy) {
        for (int gx = 0; gx < gridCols; ++gx) {
            const int xStart = xMin + gx * cellW;
            const int yStart = yMin + gy * cellH;

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
    const std::vector<GFGyroRecord>& gyroRecords,
    bool motoActionMode
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
    const double baseH = (header.height > 0) ? static_cast<double>(header.height) : 1080.0;
    const double baseLong = std::max(baseW, baseH);
    const double frameLong = static_cast<double>(std::max(frames[0].width, frames[0].height));
    const double scale = frameLong / baseLong;
    const double scaleX = static_cast<double>(frames[0].width) / baseW;
    const double scaleY = static_cast<double>(frames[0].height) / baseH;

    double rawFx = (header.lens_fx > 10.0) ? header.lens_fx : (1815.0 * (baseLong / 1920.0));
    double rawFy = (header.lens_fy > 10.0) ? header.lens_fy : rawFx;
    if (baseLong >= 1900.0 && (rawFx < 1500.0 || std::fabs(rawFx - 1660.0) < 2.0)) {
        rawFx = 1815.0;
        rawFy = 1815.0;
    } else if (baseLong >= 1200.0 && baseLong < 1500.0 && (rawFx < 1000.0 || std::fabs(rawFx - 1106.67) < 2.0)) {
        rawFx = 1210.0;
        rawFy = 1210.0;
    }
    const double fx = rawFx * scale;
    const double fy = rawFy * scale;
    const double cx = ((header.lens_cx > 10.0) ? header.lens_cx : (0.5 * baseW)) * scaleX;
    const double cy = ((header.lens_cy > 10.0) ? header.lens_cy : (0.5 * baseH)) * scaleY;

    struct VisSample {
        double tMid;
        double wx, wy, wz;
    };
    std::vector<VisSample> visSamples;
    visSamples.reserve(frames.size());

    double visEnergy = 0.0;
    for (size_t i = 1; i < frames.size(); ++i) {
        double wx = 0.0, wy = 0.0, wz = 0.0;
        if (estimateVisualOmega(frames[i - 1], frames[i], fx, fy, cx, cy, wx, wy, wz, motoActionMode)) {
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
        double dDotSum = 0.0;
        double dVisNormSq = 0.0;
        double dImuNormSq = 0.0;
        double prevIx = 0.0, prevIy = 0.0, prevIz = 0.0;
        for (size_t k = 0; k < visSamples.size(); ++k) {
            const auto& vs = visSamples[k];
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
            if (k > 0) {
                const auto& prevVs = visSamples[k - 1];
                const double dvx = vs.wx - prevVs.wx;
                const double dvy = vs.wy - prevVs.wy;
                const double dvz = vs.wz - prevVs.wz;
                const double dix = ix - prevIx;
                const double diy = iy - prevIy;
                const double diz = iz - prevIz;
                dDotSum += dvx * dix + dvy * diy + dvz * diz;
                dVisNormSq += dvx * dvx + dvy * dvy + dvz * dvz;
                dImuNormSq += dix * dix + diy * diy + diz * diz;
            }
            prevIx = ix; prevIy = iy; prevIz = iz;
        }
        const double denom = std::sqrt(visNormSq * imuNormSq);
        const double velNcc = (denom > 1e-9) ? (dotSum / denom) : 0.0;
        const double dDenom = std::sqrt(dVisNormSq * dImuNormSq);
        const double accNcc = (dDenom > 1e-6) ? (dDotSum / dDenom) : velNcc;
        return 0.70 * velNcc + 0.30 * accNcc;
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

    // IRLS Roll (Z-axis) Gyro Scale estimation in [0.92, 1.08] to eliminate 4-7 deg static tilt bias
    result.rollGyroScale = 1.0;
    result.hasRollScaleEstimate = false;
    if (bestNccOverall >= 0.70 && visSamples.size() >= 8) {
        const double* M = presets[bestPreset].m;
        const double offSec = refinedOffsetMs * 0.001;
        struct RollPair {
            double vwz, iwz, weight;
        };
        std::vector<RollPair> rPairs;
        rPairs.reserve(visSamples.size());
        double imuZEnergy = 0.0;
        for (const auto& vs : visSamples) {
            const SampledGyro raw = sampleGyroAt(sortedGyro, vs.tMid + offSec);
            const double gx = raw.gx - header.gyro_bias_x;
            const double gy = raw.gy - header.gyro_bias_y;
            const double gz = raw.gz - header.gyro_bias_z;
            const double iz = M[6] * gx + M[7] * gy + M[8] * gz;
            if (std::fabs(iz) > 0.08) {
                rPairs.push_back({vs.wz, iz, 1.0});
                imuZEnergy += iz * iz;
            }
        }
        if (rPairs.size() >= 5 && imuZEnergy > 0.03) {
            double rScale = 1.0;
            for (int iter = 0; iter < 3; ++iter) {
                double num = 0.0, den = 0.0;
                for (const auto& p : rPairs) {
                    num += p.weight * (p.vwz * p.iwz);
                    den += p.weight * (p.iwz * p.iwz);
                }
                if (den <= 1e-9) break;
                rScale = num / den;
                for (auto& p : rPairs) {
                    const double err = std::fabs(p.vwz - rScale * p.iwz);
                    p.weight = (err < 0.04) ? 1.0 : (0.04 / (err + 1e-6));
                }
            }
            if (std::isfinite(rScale)) {
                result.rollGyroScale = std::max(0.92, std::min(1.08, rScale));
                result.hasRollScaleEstimate = true;
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

            const bool scanAlongX = (w < h);
            auto solveHalf = [&](int halfIdx, double& outScanFrac, double& owx, double& owy, double& owz) -> bool {
                const int margin = 20;
                const int gridCols = 6;
                const int gridRows = 4;
                const int cellW = (w - 2 * margin) / gridCols;
                const int cellH = (h - 2 * margin) / gridRows;
                const int gxStart = scanAlongX ? (halfIdx == 0 ? 0 : 3) : 0;
                const int gxEnd = scanAlongX ? (halfIdx == 0 ? 3 : 6) : gridCols;
                const int gyStart = scanAlongX ? 0 : (halfIdx == 0 ? 0 : 2);
                const int gyEnd = scanAlongX ? gridRows : (halfIdx == 0 ? 2 : 4);
                struct HFlow { double x, y, dx, dy; };
                std::vector<HFlow> hobs;
                double scanFracSum = 0.0;
                for (int gy = gyStart; gy < gyEnd; ++gy) {
                    for (int gx = gxStart; gx < gxEnd; ++gx) {
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
                        scanFracSum += scanAlongX
                            ? ((static_cast<double>(bestU) / static_cast<double>(w - 1)) - 0.5)
                            : ((static_cast<double>(bestV) / static_cast<double>(h - 1)) - 0.5);
                    }
                }
                if (hobs.size() < 4) return false;
                outScanFrac = scanFracSum / static_cast<double>(hobs.size());
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
            if (solveHalf(0, hv.yFracTop, hv.wxTop, hv.wyTop, hv.wzTop) &&
                solveHalf(1, hv.yFracBot, hv.wxBot, hv.wyBot, hv.wzBot)) {
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

void HybridResidualTracker::reset(const CalibrationProfile& profile, const TrajectoryResult& traj, bool motoActionMode) {
    profile_ = profile;
    rawQuats_ = traj.rawFrameQuats;
    smoothQuats_ = traj.smoothFrameQuats;
    prevFrame_ = OpticalFlowFrame{};
    prevQCorr_ = Quat::identity();
    hasPrev_ = false;
    motoActionMode_ = motoActionMode;
    frameIdx_ = 0;
    deltaU_.clear();
    deltaV_.clear();
    deltaU_.reserve(rawQuats_.size());
    deltaV_.reserve(rawQuats_.size());
}

void HybridResidualTracker::pushFrame(const OpticalFlowFrame& frame) {
    if (motoActionMode_) {
        ++frameIdx_;
        return;
    }
    const size_t nTraj = std::min(rawQuats_.size(), smoothQuats_.size());
    Quat currQCorr = Quat::identity();
    if (nTraj > 0) {
        const size_t qIdx = std::min(frameIdx_, nTraj - 1);
        currQCorr = (rawQuats_[qIdx].conjugate() * smoothQuats_[qIdx]).normalized();
    }

    double estDu = 0.0;
    double estDv = 0.0;

    if (hasPrev_ && frame.width >= 64 && frame.height >= 64 &&
        frame.width == prevFrame_.width && frame.height == prevFrame_.height &&
        frame.luma.size() == static_cast<size_t>(frame.width * frame.height) &&
        prevFrame_.luma.size() == static_cast<size_t>(prevFrame_.width * prevFrame_.height) &&
        profile_.width > 0 && profile_.height > 0) {

        const int w = frame.width;
        const int h = frame.height;
        int hw = 0, hh = 0;
        const std::vector<uint8_t> h0 = downsample2x(prevFrame_.luma, w, h, hw, hh);
        const std::vector<uint8_t> h1 = downsample2x(frame.luma, w, h, hw, hh);

        const double scaleX = static_cast<double>(w) / static_cast<double>(profile_.width);
        const double scaleY = static_cast<double>(h) / static_cast<double>(profile_.height);
        const double fxS = profile_.fx * scaleX;
        const double fyS = profile_.fy * scaleY;
        const double cxS = profile_.cx * scaleX;
        const double cyS = profile_.cy * scaleY;

        double R0[9], R1[9];
        FrameWarpCalculator::computeRotationMatrix3x3(prevQCorr_.conjugate(), R0);
        FrameWarpCalculator::computeRotationMatrix3x3(currQCorr.conjugate(), R1);

        auto projectVirt = [&](double u, double v, const double R[9]) -> std::pair<double, double> {
            const double xn = (u - cxS) / fxS;
            const double yn = (v - cyS) / fyS;
            const double rx = R[0] * xn + R[1] * yn + R[2];
            const double ry = R[3] * xn + R[4] * yn + R[5];
            const double rz = R[6] * xn + R[7] * yn + R[8];
            const double invZ = 1.0 / std::max(0.15, rz);
            return {fxS * (rx * invZ) + cxS, fyS * (ry * invZ) + cyS};
        };

        const int margin = 20;
        const int gridCols = 6;
        const int gridRows = 4;
        const int cellW = (w - 2 * margin) / gridCols;
        const int cellH = (h - 2 * margin) / gridRows;

        std::vector<double> duList;
        std::vector<double> dvList;
        duList.reserve(gridCols * gridRows);
        dvList.reserve(gridCols * gridRows);

        for (int gy = 0; gy < gridRows; ++gy) {
            for (int gx = 0; gx < gridCols; ++gx) {
                const int xStart = margin + gx * cellW;
                const int yStart = margin + gy * cellH;

                int bestU = xStart + cellW / 2;
                int bestV = yStart + cellH / 2;
                int bestScore = -1;
                for (int v = yStart + 2; v < yStart + cellH - 2; v += 2) {
                    for (int u = xStart + 2; u < xStart + cellW - 2; u += 2) {
                        const int ix = static_cast<int>(prevFrame_.luma[v * w + u + 1]) - static_cast<int>(prevFrame_.luma[v * w + u - 1]);
                        const int iy = static_cast<int>(prevFrame_.luma[(v + 1) * w + u]) - static_cast<int>(prevFrame_.luma[(v - 1) * w + u]);
                        const int score = std::min(std::abs(ix), std::abs(iy));
                        if (score > bestScore) {
                            bestScore = score;
                            bestU = u;
                            bestV = v;
                        }
                    }
                }
                if (bestScore < 5) {
                    continue;
                }

                const int hu0 = bestU / 2;
                const int hv0 = bestV / 2;
                const int cRad = 3;
                const int cSearch = 12;
                if (hu0 - cRad - cSearch < 1 || hu0 + cRad + cSearch >= hw - 1 ||
                    hv0 - cRad - cSearch < 1 || hv0 + cRad + cSearch >= hh - 1) {
                    continue;
                }

                int bestHu1 = hu0, bestHv1 = hv0, bestCSad = 1 << 30;
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

                int bestFu1 = fCenterU, bestFv1 = fCenterV, bestFSad = 1 << 30;
                for (int dv = -fSearch; dv <= fSearch; ++dv) {
                    for (int du = -fSearch; du <= fSearch; ++du) {
                        const int sad = computeSadPatch(prevFrame_.luma.data(), w, frame.luma.data(), w, bestU, bestV, fCenterU + du, fCenterV + dv, fRad);
                        if (sad < bestFSad) {
                            bestFSad = sad;
                            bestFu1 = fCenterU + du;
                            bestFv1 = fCenterV + dv;
                        }
                    }
                }

                double subU = static_cast<double>(bestFu1);
                double subV = static_cast<double>(bestFv1);
                if (bestFu1 - fRad - 1 >= 0 && bestFu1 + fRad + 1 < w) {
                    const int sL = computeSadPatch(prevFrame_.luma.data(), w, frame.luma.data(), w, bestU, bestV, bestFu1 - 1, bestFv1, fRad);
                    const int sR = computeSadPatch(prevFrame_.luma.data(), w, frame.luma.data(), w, bestU, bestV, bestFu1 + 1, bestFv1, fRad);
                    const double denom = 2.0 * (sL - 2.0 * bestFSad + sR);
                    if (std::fabs(denom) > 1e-6) {
                        subU += std::min(0.5, std::max(-0.5, (sL - sR) / denom));
                    }
                }
                if (bestFv1 - fRad - 1 >= 0 && bestFv1 + fRad + 1 < h) {
                    const int sU = computeSadPatch(prevFrame_.luma.data(), w, frame.luma.data(), w, bestU, bestV, bestFu1, bestFv1 - 1, fRad);
                    const int sD = computeSadPatch(prevFrame_.luma.data(), w, frame.luma.data(), w, bestU, bestV, bestFu1, bestFv1 + 1, fRad);
                    const double denom = 2.0 * (sU - 2.0 * bestFSad + sD);
                    if (std::fabs(denom) > 1e-6) {
                        subV += std::min(0.5, std::max(-0.5, (sU - sD) / denom));
                    }
                }

                const auto p0 = projectVirt(static_cast<double>(bestU), static_cast<double>(bestV), R0);
                const auto p1 = projectVirt(subU, subV, R1);
                duList.push_back((p1.first - p0.first) / scaleX);
                dvList.push_back((p1.second - p0.second) / scaleY);
            }
        }

        if (duList.size() >= 4) {
            std::vector<double> sortedU = duList;
            std::vector<double> sortedV = dvList;
            std::sort(sortedU.begin(), sortedU.end());
            std::sort(sortedV.begin(), sortedV.end());
            double muU = sortedU[sortedU.size() / 2];
            double muV = sortedV[sortedV.size() / 2];

            // 3-iteration IRLS Huber mean to reject foreground moving objects
            for (int iter = 0; iter < 3; ++iter) {
                double sumW = 0.0, sumU = 0.0, sumV = 0.0;
                for (size_t k = 0; k < duList.size(); ++k) {
                    const double err = std::hypot(duList[k] - muU, dvList[k] - muV);
                    const double wHuber = (err <= 2.5) ? 1.0 : (2.5 / (err + 1e-6));
                    sumW += wHuber;
                    sumU += wHuber * duList[k];
                    sumV += wHuber * dvList[k];
                }
                if (sumW > 1e-6) {
                    muU = sumU / sumW;
                    muV = sumV / sumW;
                }
            }
            estDu = muU;
            estDv = muV;
        }
    }

    if (hasPrev_) {
        deltaU_.push_back(estDu);
        deltaV_.push_back(estDv);
    }

    prevFrame_ = frame;
    prevQCorr_ = currQCorr;
    hasPrev_ = true;
    ++frameIdx_;
}

HybridResidualResult HybridResidualTracker::finalize(double nominalFps) const {
    HybridResidualResult res;
    const size_t n = std::min(rawQuats_.size(), smoothQuats_.size());
    if (n < 3 || deltaU_.empty()) {
        return res;
    }

    const size_t m = std::min(n, deltaU_.size() + 1);
    std::vector<double> rawTx(n, 0.0);
    std::vector<double> rawTy(n, 0.0);
    for (size_t i = 1; i < m; ++i) {
        rawTx[i] = rawTx[i - 1] + deltaU_[i - 1];
        rawTy[i] = rawTy[i - 1] + deltaV_[i - 1];
    }
    for (size_t i = m; i < n; ++i) {
        rawTx[i] = rawTx[m - 1];
        rawTy[i] = rawTy[m - 1];
    }

    // Zero-Phase High-Pass Filter (tau = 1.20s -> handles both bounce and off-axis 90 deg roll lever-arm drift)
    const double dt = (nominalFps > 1.0) ? (1.0 / nominalFps) : (1.0 / 30.0);
    const double alpha = std::exp(-dt / 1.20);

    auto zeroPhaseLowPass = [&](const std::vector<double>& in) -> std::vector<double> {
        std::vector<double> fwd(n, in[0]);
        for (size_t i = 1; i < n; ++i) {
            fwd[i] = alpha * fwd[i - 1] + (1.0 - alpha) * in[i];
        }
        std::vector<double> bwd = fwd;
        for (size_t i = n - 1; i > 0; --i) {
            const size_t idx = i - 1;
            bwd[idx] = alpha * bwd[idx + 1] + (1.0 - alpha) * fwd[idx];
        }
        return bwd;
    };

    const std::vector<double> baseTx = zeroPhaseLowPass(rawTx);
    const std::vector<double> baseTy = zeroPhaseLowPass(rawTy);

    const double maxShiftPx = 85.0 * (static_cast<double>(profile_.height > 0 ? profile_.height : 1080) / 1080.0);
    res.shiftX.resize(n, 0.0);
    res.shiftY.resize(n, 0.0);

    double sumSq = 0.0;
    double maxAmp = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double hpX = rawTx[i] - baseTx[i];
        const double hpY = rawTy[i] - baseTy[i];
        const double clampedX = maxShiftPx * std::tanh(hpX / maxShiftPx);
        const double clampedY = maxShiftPx * std::tanh(hpY / maxShiftPx);
        res.shiftX[i] = -clampedX;
        res.shiftY[i] = -clampedY;
        const double amp = std::hypot(clampedX, clampedY);
        maxAmp = std::max(maxAmp, amp);
        sumSq += amp * amp;
    }

    res.maxAmplitudePx = maxAmp;
    res.rmsResidualPx = std::sqrt(sumSq / static_cast<double>(n));
    res.valid = true;
    return res;
}

HybridResidualResult OpticalSync::estimateHybridResidualShifts(
    const std::vector<OpticalFlowFrame>& frames,
    const CalibrationProfile& profile,
    const TrajectoryResult& traj,
    double nominalFps
) {
    HybridResidualTracker tracker;
    tracker.reset(profile, traj);
    for (const auto& f : frames) {
        tracker.pushFrame(f);
    }
    return tracker.finalize(nominalFps);
}

} // namespace gf
