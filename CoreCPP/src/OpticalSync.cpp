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
                1.0
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

    double refinedOffsetMs = -80.0 + bestOffIdx * 2.0;
    if (bestOffIdx > 0 && bestOffIdx < numOffsets - 1) {
        const double yL = bestPresetCurve[bestOffIdx - 1];
        const double yC = bestPresetCurve[bestOffIdx];
        const double yR = bestPresetCurve[bestOffIdx + 1];
        const double denom = 2.0 * (yL - 2.0 * yC + yR);
        if (std::fabs(denom) > 1e-9) {
            const double delta = std::min(0.5, std::max(-0.5, (yL - yR) / denom));
            refinedOffsetMs += delta * 2.0;
        }
    }

    result.valid = true;
    result.bestOffsetMs = refinedOffsetMs;
    result.bestNCC = bestNccOverall;
    result.bestAxisPresetIndex = bestPreset;
    result.axisPresetName = presets[bestPreset].name;
    std::copy(std::begin(presets[bestPreset].m), std::end(presets[bestPreset].m), std::begin(result.axisMatrix));

    char buf[160];
    std::snprintf(
        buf, sizeof(buf),
        "Auto-Sync: %+.1f ms | NCC: %.3f | Axis: %s",
        result.bestOffsetMs, result.bestNCC, result.axisPresetName.c_str()
    );
    result.diagnosticMessage = buf;
    return result;
}

} // namespace gf
