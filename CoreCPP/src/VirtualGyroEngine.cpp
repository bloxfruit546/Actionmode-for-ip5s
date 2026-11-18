#include "VirtualGyroEngine.hpp"
#include "SidecarIO.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace gf {

namespace {

inline float sampleBilinear(const uint8_t* img, int w, int h, float x, float y) {
    if (x < 0.0f) x = 0.0f;
    if (y < 0.0f) y = 0.0f;
    if (x > static_cast<float>(w - 1)) x = static_cast<float>(w - 1);
    if (y > static_cast<float>(h - 1)) y = static_cast<float>(h - 1);

    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = std::min(x0 + 1, w - 1);
    const int y1 = std::min(y0 + 1, h - 1);
    const float fx = x - static_cast<float>(x0);
    const float fy = y - static_cast<float>(y0);

    const float i00 = static_cast<float>(img[y0 * w + x0]);
    const float i10 = static_cast<float>(img[y0 * w + x1]);
    const float i01 = static_cast<float>(img[y1 * w + x0]);
    const float i11 = static_cast<float>(img[y1 * w + x1]);

    const float top = i00 + fx * (i10 - i00);
    const float bot = i01 + fx * (i11 - i01);
    return top + fy * (bot - top);
}

bool solve3x3Linear(double A[3][3], double b[3], double x[3]) {
    double M[3][4];
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            M[r][c] = A[r][c];
        }
        M[r][3] = b[r];
    }

    for (int col = 0; col < 3; ++col) {
        int piv = col;
        double maxVal = std::fabs(M[col][col]);
        for (int r = col + 1; r < 3; ++r) {
            const double v = std::fabs(M[r][col]);
            if (v > maxVal) {
                maxVal = v;
                piv = r;
            }
        }
        if (maxVal < 1e-12) {
            return false;
        }
        if (piv != col) {
            for (int c = col; c < 4; ++c) {
                std::swap(M[col][c], M[piv][c]);
            }
        }
        const double invPiv = 1.0 / M[col][col];
        for (int r = col + 1; r < 3; ++r) {
            const double f = M[r][col] * invPiv;
            for (int c = col; c < 4; ++c) {
                M[r][c] -= f * M[col][c];
            }
        }
    }

    for (int r = 2; r >= 0; --r) {
        double sum = M[r][3];
        for (int c = r + 1; c < 3; ++c) {
            sum -= M[r][c] * x[c];
        }
        x[r] = sum / M[r][r];
    }
    return true;
}

bool trackSinglePointLK(
    const uint8_t* prevImg, const uint8_t* currImg,
    int w, int h,
    float px, float py,
    float& cx, float& cy
) {
    constexpr int kRad = 3; // 7x7 window
    constexpr int kMaxIter = 5;

    if (px < kRad + 1 || px >= w - kRad - 2 || py < kRad + 1 || py >= h - kRad - 2) {
        return false;
    }

    // Precompute spatial gradients and structure tensor G on prevImg around (px, py)
    double gxx = 0.0, gxy = 0.0, gyy = 0.0;
    float patchI[49];
    float patchIx[49];
    float patchIy[49];
    int idx = 0;

    for (int dy = -kRad; dy <= kRad; ++dy) {
        for (int dx = -kRad; dx <= kRad; ++dx) {
            const float sx = px + static_cast<float>(dx);
            const float sy = py + static_cast<float>(dy);
            const float val = sampleBilinear(prevImg, w, h, sx, sy);
            const float ix = 0.5f * (sampleBilinear(prevImg, w, h, sx + 1.0f, sy) -
                                     sampleBilinear(prevImg, w, h, sx - 1.0f, sy));
            const float iy = 0.5f * (sampleBilinear(prevImg, w, h, sx, sy + 1.0f) -
                                     sampleBilinear(prevImg, w, h, sx, sy - 1.0f));
            patchI[idx] = val;
            patchIx[idx] = ix;
            patchIy[idx] = iy;
            gxx += ix * ix;
            gxy += ix * iy;
            gyy += iy * iy;
            ++idx;
        }
    }

    const double det = gxx * gyy - gxy * gxy;
    if (det < 10.0) {
        return false;
    }
    const double invDet = 1.0 / det;

    for (int iter = 0; iter < kMaxIter; ++iter) {
        if (cx < kRad + 1 || cx >= w - kRad - 2 || cy < kRad + 1 || cy >= h - kRad - 2) {
            return false;
        }
        double bx = 0.0, by = 0.0;
        idx = 0;
        for (int dy = -kRad; dy <= kRad; ++dy) {
            for (int dx = -kRad; dx <= kRad; ++dx) {
                const float jVal = sampleBilinear(currImg, w, h, cx + static_cast<float>(dx), cy + static_cast<float>(dy));
                const double diff = static_cast<double>(patchI[idx] - jVal);
                bx += diff * patchIx[idx];
                by += diff * patchIy[idx];
                ++idx;
            }
        }
        const float stepX = static_cast<float>((gyy * bx - gxy * by) * invDet);
        const float stepY = static_cast<float>((gxx * by - gxy * bx) * invDet);
        cx += stepX;
        cy += stepY;
        if (stepX * stepX + stepY * stepY < 1e-4f) {
            break;
        }
    }
    return (cx >= 1.0f && cx < w - 2.0f && cy >= 1.0f && cy < h - 2.0f);
}

bool trackPyramidalLK(
    const uint8_t* const prevLevels[3], const uint8_t* const currLevels[3],
    const int widths[3], const int heights[3],
    float x0, float y0, float& outX1, float& outY1
) {
    float cx = x0 * 0.25f;
    float cy = y0 * 0.25f;

    for (int lvl = 2; lvl >= 0; --lvl) {
        const float scale = (lvl == 2) ? 0.25f : ((lvl == 1) ? 0.5f : 1.0f);
        const float px = x0 * scale;
        const float py = y0 * scale;
        if (!trackSinglePointLK(prevLevels[lvl], currLevels[lvl], widths[lvl], heights[lvl], px, py, cx, cy)) {
            return false;
        }
        if (lvl > 0) {
            cx *= 2.0f;
            cy *= 2.0f;
        }
    }

    outX1 = cx;
    outY1 = cy;
    return true;
}

} // namespace

VirtualGyroEngine::VirtualGyroEngine(const VirtualGyroConfig& config)
    : config_(config) {
}

void VirtualGyroEngine::buildPyramid(const uint8_t* luma, int width, int height, double ts, FramePyramid& outPyr) {
    outPyr.timestampSec = ts;
    outPyr.levels[0].width = width;
    outPyr.levels[0].height = height;
    outPyr.levels[0].data.assign(luma, luma + static_cast<size_t>(width) * height);

    for (int lvl = 1; lvl < 3; ++lvl) {
        const auto& src = outPyr.levels[lvl - 1];
        auto& dst = outPyr.levels[lvl];
        dst.width = std::max(1, src.width / 2);
        dst.height = std::max(1, src.height / 2);
        dst.data.resize(static_cast<size_t>(dst.width) * dst.height);
        for (int y = 0; y < dst.height; ++y) {
            const int sy0 = std::min(y * 2, src.height - 1);
            const int sy1 = std::min(sy0 + 1, src.height - 1);
            for (int x = 0; x < dst.width; ++x) {
                const int sx0 = std::min(x * 2, src.width - 1);
                const int sx1 = std::min(sx0 + 1, src.width - 1);
                const int sum = src.data[sy0 * src.width + sx0] +
                                src.data[sy0 * src.width + sx1] +
                                src.data[sy1 * src.width + sx0] +
                                src.data[sy1 * src.width + sx1];
                dst.data[y * dst.width + x] = static_cast<uint8_t>((sum + 2) >> 2);
            }
        }
    }
}

bool VirtualGyroEngine::pushLumaFrame(const uint8_t* luma, int width, int height, double timestampSec) {
    if (!luma || width < 32 || height < 32) {
        return false;
    }

    GFFrameRecord rec{};
    rec.frame_index = static_cast<uint32_t>(frames_.size());
    rec.pts_sec = timestampSec;
    rec.exposure_duration_sec = 0.01f;
    rec.iso = 100.0f;
    rec.lens_position = 0.5f;
    rec.rolling_shutter_sec = static_cast<float>(config_.readoutTimeSec);
    frames_.push_back(rec);

    if (!hasPrevFrame_) {
        buildPyramid(luma, width, height, timestampSec, prevPyr_);
        hasPrevFrame_ = true;
        return true;
    }

    buildPyramid(luma, width, height, timestampSec, currPyr_);

    VisualOmegaSample sample{};
    double inlierRatio = 0.0;
    if (trackAndSolveRotation(prevPyr_, currPyr_, sample, inlierRatio)) {
        gyroSamples_.push_back(sample);
        inlierRatioSum_ += inlierRatio;
        ++inlierRatioCount_;
    } else {
        VisualOmegaSample fallback{};
        fallback.timestampSec = 0.5 * (prevPyr_.timestampSec + currPyr_.timestampSec);
        gyroSamples_.push_back(fallback);
    }

    std::swap(prevPyr_, currPyr_);
    return true;
}

bool VirtualGyroEngine::trackAndSolveRotation(
    const FramePyramid& prev, const FramePyramid& curr,
    VisualOmegaSample& outSample, double& outInlierRatio
) const {
    const int w = prev.levels[0].width;
    const int h = prev.levels[0].height;
    if (w != curr.levels[0].width || h != curr.levels[0].height) {
        return false;
    }

    const double dt = curr.timestampSec - prev.timestampSec;
    if (dt <= 1e-4 || dt > 0.25) {
        return false;
    }

    const double baseW = (config_.videoWidth > 0) ? static_cast<double>(config_.videoWidth) : 1920.0;
    const double scale = static_cast<double>(w) / baseW;
    const double fx = ((config_.lensFx > 10.0) ? config_.lensFx : 1660.0) * scale;
    const double fy = ((config_.lensFy > 10.0) ? config_.lensFy : 1660.0) * scale;
    const double cx = ((config_.lensCx > 10.0) ? config_.lensCx : 960.0) * scale;
    const double cy = ((config_.lensCy > 10.0) ? config_.lensCy : 540.0) * scale;

    const uint8_t* prevLvls[3] = {prev.levels[0].data.data(), prev.levels[1].data.data(), prev.levels[2].data.data()};
    const uint8_t* currLvls[3] = {curr.levels[0].data.data(), curr.levels[1].data.data(), curr.levels[2].data.data()};
    const int widths[3] = {prev.levels[0].width, prev.levels[1].width, prev.levels[2].width};
    const int heights[3] = {prev.levels[0].height, prev.levels[1].height, prev.levels[2].height};

    // 8x6 grid, up to 2 strongest corner-like features per cell (<= 96 features)
    constexpr int kCols = 8;
    constexpr int kRows = 6;
    const int margin = std::max(14, std::min(w, h) / 16);
    const int cellW = (w - 2 * margin) / kCols;
    const int cellH = (h - 2 * margin) / kRows;
    if (cellW < 8 || cellH < 8) {
        return false;
    }

    struct TrackObs {
        double u, v, du, dv;
        double x, y, dx, dy;
    };
    std::vector<TrackObs> obs;
    obs.reserve(kCols * kRows * 2);

    const uint8_t* p0 = prevLvls[0];
    for (int gy = 0; gy < kRows; ++gy) {
        for (int gx = 0; gx < kCols; ++gx) {
            const int xStart = margin + gx * cellW;
            const int yStart = margin + gy * cellH;
            int bestU[2] = {-1, -1};
            int bestV[2] = {-1, -1};
            int bestScore[2] = {-1, -1};

            for (int v = yStart + 2; v < yStart + cellH - 2; v += 2) {
                for (int u = xStart + 2; u < xStart + cellW - 2; u += 2) {
                    const int ix = static_cast<int>(p0[v * w + u + 1]) - static_cast<int>(p0[v * w + u - 1]);
                    const int iy = static_cast<int>(p0[(v + 1) * w + u]) - static_cast<int>(p0[(v - 1) * w + u]);
                    const int score = std::min(std::abs(ix), std::abs(iy));
                    if (score < 6) continue;

                    if (score > bestScore[0]) {
                        if (bestU[0] < 0 || std::abs(u - bestU[0]) + std::abs(v - bestV[0]) > 6) {
                            bestScore[1] = bestScore[0];
                            bestU[1] = bestU[0];
                            bestV[1] = bestV[0];
                        }
                        bestScore[0] = score;
                        bestU[0] = u;
                        bestV[0] = v;
                    } else if (score > bestScore[1] &&
                               (bestU[0] < 0 || std::abs(u - bestU[0]) + std::abs(v - bestV[0]) > 6)) {
                        bestScore[1] = score;
                        bestU[1] = u;
                        bestV[1] = v;
                    }
                }
            }

            for (int k = 0; k < 2; ++k) {
                if (bestScore[k] < 6 || bestU[k] < 0) continue;
                const float u0 = static_cast<float>(bestU[k]);
                const float v0 = static_cast<float>(bestV[k]);
                float u1 = u0, v1 = v0;
                if (!trackPyramidalLK(prevLvls, currLvls, widths, heights, u0, v0, u1, v1)) {
                    continue;
                }
                // Forward-Backward consistency check (< 1.0 px)
                float uBack = u1, vBack = v1;
                if (!trackPyramidalLK(currLvls, prevLvls, widths, heights, u1, v1, uBack, vBack)) {
                    continue;
                }
                if (std::hypot(uBack - u0, vBack - v0) >= 1.0f) {
                    continue;
                }

                const double du = static_cast<double>(u1 - u0);
                const double dv = static_cast<double>(v1 - v0);
                obs.push_back({
                    static_cast<double>(u0),
                    static_cast<double>(v0),
                    du,
                    dv,
                    (static_cast<double>(u0) - cx) / fx,
                    (static_cast<double>(v0) - cy) / fy,
                    du / fx,
                    dv / fy
                });
            }
        }
    }

    if (obs.size() < 6) {
        return false;
    }

    // 80-iteration deterministic RANSAC for 3-DOF rotational homography (1.5 px threshold)
    constexpr int kRansacIters = 80;
    constexpr double kInlierThreshPx = 1.5;
    std::vector<int> bestInliers;
    bestInliers.reserve(obs.size());
    uint32_t rng = 0x9E3779B9u ^ static_cast<uint32_t>(obs.size());

    auto nextRand = [&]() -> size_t {
        rng ^= rng << 13;
        rng ^= rng >> 17;
        rng ^= rng << 5;
        return static_cast<size_t>(rng) % obs.size();
    };

    for (int iter = 0; iter < kRansacIters; ++iter) {
        const size_t i0 = nextRand();
        size_t i1 = nextRand();
        if (i1 == i0) i1 = (i0 + 1) % obs.size();

        double ATA[3][3] = {};
        double ATb[3] = {};
        const size_t pair[2] = {i0, i1};
        for (size_t idx : pair) {
            const auto& o = obs[idx];
            const double rx[3] = {o.x * o.y, -(1.0 + o.x * o.x), o.y};
            const double ry[3] = {(1.0 + o.y * o.y), -o.x * o.y, -o.x};
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    ATA[r][c] += rx[r] * rx[c] + ry[r] * ry[c];
                }
                ATb[r] += rx[r] * o.dx + ry[r] * o.dy;
            }
        }

        double omegaDt[3] = {};
        if (!solve3x3Linear(ATA, ATb, omegaDt)) {
            continue;
        }

        std::vector<int> inliers;
        inliers.reserve(obs.size());
        for (size_t i = 0; i < obs.size(); ++i) {
            const auto& o = obs[i];
            const double predDu = fx * (omegaDt[0] * (o.x * o.y) - omegaDt[1] * (1.0 + o.x * o.x) + omegaDt[2] * o.y);
            const double predDv = fy * (omegaDt[0] * (1.0 + o.y * o.y) - omegaDt[1] * (o.x * o.y) - omegaDt[2] * o.x);
            if (std::hypot(o.du - predDu, o.dv - predDv) < kInlierThreshPx) {
                inliers.push_back(static_cast<int>(i));
            }
        }

        if (inliers.size() > bestInliers.size()) {
            bestInliers = std::move(inliers);
        }
    }

    if (bestInliers.size() < 4) {
        return false;
    }

    // Refit least squares on all RANSAC inliers
    double ATA[3][3] = {};
    double ATb[3] = {};
    for (int idx : bestInliers) {
        const auto& o = obs[static_cast<size_t>(idx)];
        const double rx[3] = {o.x * o.y, -(1.0 + o.x * o.x), o.y};
        const double ry[3] = {(1.0 + o.y * o.y), -o.x * o.y, -o.x};
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                ATA[r][c] += rx[r] * rx[c] + ry[r] * ry[c];
            }
            ATb[r] += rx[r] * o.dx + ry[r] * o.dy;
        }
    }

    double omegaDt[3] = {};
    if (!solve3x3Linear(ATA, ATb, omegaDt)) {
        return false;
    }

    outSample.timestampSec = 0.5 * (prev.timestampSec + curr.timestampSec);
    outSample.wxCam = omegaDt[0] / dt;
    outSample.wyCam = omegaDt[1] / dt;
    outSample.wzCam = omegaDt[2] / dt;
    outInlierRatio = static_cast<double>(bestInliers.size()) / static_cast<double>(obs.size());
    return true;
}

VirtualGyroResult VirtualGyroEngine::finalizeAndWrite(const std::string& outputSidecarPath) {
    VirtualGyroResult res;
    if (frames_.size() < 2 || gyroSamples_.empty()) {
        res.errorMessage = "Insufficient frames to generate Virtual Gyro sidecar.";
        return res;
    }

    GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.capture_mode = (config_.nominalFps > 60.0) ? 1u : 0u;
    hdr.width = config_.videoWidth;
    hdr.height = config_.videoHeight;
    hdr.nominal_fps = config_.nominalFps;
    hdr.readout_time_sec = config_.readoutTimeSec;
    hdr.gyro_bias_x = 0.0;
    hdr.gyro_bias_y = 0.0;
    hdr.gyro_bias_z = 0.0;
    hdr.lens_fx = config_.lensFx;
    hdr.lens_fy = config_.lensFy;
    hdr.lens_cx = config_.lensCx;
    hdr.lens_cy = config_.lensCy;
    hdr.dist_k1 = config_.distK1;
    hdr.dist_k2 = config_.distK2;
    hdr.dist_p1 = config_.distP1;
    hdr.dist_p2 = config_.distP2;
    std::snprintf(hdr.uuid, sizeof(hdr.uuid), "%s", config_.uuid.c_str());
    hdr.reserved[0] = 1; // Flag: Virtual Gyro (Optical RANSAC)

    SidecarWriter writer;
    if (!writer.open(outputSidecarPath, hdr)) {
        res.errorMessage = "Failed to open output sidecar path: " + outputSidecarPath;
        return res;
    }

    // Pad boundary gyro samples so TrajectoryEngine covers [firstPts - 0.02, lastPts + 0.02]
    auto writeCamGyroAsLandscapeLeft = [&](double ts, double wxCam, double wyCam, double wzCam) {
        GFGyroRecord grec{};
        grec.timestamp_sec = ts;
        // LandscapeLeft (-y, -x, -z) maps (-wyCam, -wxCam, -wzCam) -> (wxCam, wyCam, wzCam)
        grec.gx = -wyCam;
        grec.gy = -wxCam;
        grec.gz = -wzCam;
        writer.writeGyro(grec);

        GFAccelRecord arec{};
        arec.timestamp_sec = ts;
        // LandscapeLeft (-y, -x, -z) maps (0, -1, 0) -> (0, +1, 0) gravity
        arec.ax = 0.0f;
        arec.ay = -1.0f;
        arec.az = 0.0f;
        writer.writeAccel(arec);
    };

    const double firstPts = frames_.front().pts_sec;
    const double lastPts = frames_.back().pts_sec;
    writeCamGyroAsLandscapeLeft(firstPts - 0.02, 0.0, 0.0, 0.0);
    for (const auto& gs : gyroSamples_) {
        writeCamGyroAsLandscapeLeft(gs.timestampSec, gs.wxCam, gs.wyCam, gs.wzCam);
    }
    writeCamGyroAsLandscapeLeft(lastPts + 0.02, 0.0, 0.0, 0.0);

    for (const auto& fr : frames_) {
        writer.writeFrame(fr);
    }

    const double duration = std::max(0.0, lastPts - firstPts);
    if (!writer.closeWithFooter(duration)) {
        res.errorMessage = "Failed to finalize Virtual Gyro sidecar footer.";
        return res;
    }

    res.success = true;
    res.frameCount = frames_.size();
    res.gyroCount = gyroSamples_.size() + 2;
    res.meanInlierRatio = (inlierRatioCount_ > 0) ? (inlierRatioSum_ / static_cast<double>(inlierRatioCount_)) : 0.0;
    return res;
}

} // namespace gf
