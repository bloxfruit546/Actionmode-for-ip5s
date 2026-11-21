#include "TranslationalStabilizer2D.hpp"
#include "FrameWarpCalculator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#ifdef __APPLE__
#include <Accelerate/Accelerate.h>
#endif

namespace gf {

namespace {

void downsampleBoxAvg(
    const uint8_t* src, int srcW, int srcH, int srcStride,
    uint8_t* dst, int dstW, int dstH
) {
    for (int dy = 0; dy < dstH; ++dy) {
        const int y0 = (dy * srcH) / dstH;
        const int y1 = std::max(y0 + 1, ((dy + 1) * srcH) / dstH);
        uint8_t* dstRow = dst + dy * dstW;
        for (int dx = 0; dx < dstW; ++dx) {
            const int x0 = (dx * srcW) / dstW;
            const int x1 = std::max(x0 + 1, ((dx + 1) * srcW) / dstW);
            uint32_t sum = 0;
            uint32_t cnt = 0;
            for (int sy = y0; sy < y1; ++sy) {
                const uint8_t* sRow = src + sy * srcStride;
                for (int sx = x0; sx < x1; ++sx) {
                    sum += sRow[sx];
                    ++cnt;
                }
            }
            dstRow[dx] = static_cast<uint8_t>(sum / std::max(1u, cnt));
        }
    }
}

inline double sampleBilinearU8(const uint8_t* img, int w, double x, double y) {
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const double fx = x - x0;
    const double fy = y - y0;
    const uint8_t* r0 = img + y0 * w;
    const uint8_t* r1 = img + (y0 + 1) * w;
    const double v0 = r0[x0] * (1.0 - fx) + r0[x0 + 1] * fx;
    const double v1 = r1[x0] * (1.0 - fx) + r1[x0 + 1] * fx;
    return v0 * (1.0 - fy) + v1 * fy;
}

void rectifyUnzoomedFrame(
    const uint8_t* srcDown, int w, int h,
    const Quat& qCorr,
    double fxS, double fyS, double cxS, double cyS,
    uint8_t* outRect, uint8_t* outMask
) {
    float H[9];
    // Zoom = 1.0 unzoomed gyro rectification
    FrameWarpCalculator::computeInverseHomography(qCorr, fxS, fyS, cxS, cyS, 1.0, H);

    const double minX = 2.0;
    const double maxX = static_cast<double>(w - 3);
    const double minY = 2.0;
    const double maxY = static_cast<double>(h - 3);

    for (int y = 0; y < h; ++y) {
        const double py = static_cast<double>(y) + 0.5;
        uint8_t* rOut = outRect + y * w;
        uint8_t* mOut = outMask + y * w;
        for (int x = 0; x < w; ++x) {
            const double px = static_cast<double>(x) + 0.5;
            const double wVal = H[6] * px + H[7] * py + H[8];
            if (wVal <= 0.15) {
                rOut[x] = 0;
                mOut[x] = 0;
                continue;
            }
            const double sx = (H[0] * px + H[1] * py + H[2]) / wVal - 0.5;
            const double sy = (H[3] * px + H[4] * py + H[5]) / wVal - 0.5;
            if (sx < minX || sx > maxX || sy < minY || sy > maxY) {
                rOut[x] = 0;
                mOut[x] = 0;
                continue;
            }
            const double val = sampleBilinearU8(srcDown, w, sx, sy);
            rOut[x] = static_cast<uint8_t>(std::min(255.0, std::max(0.0, val + 0.5)));
            mOut[x] = 1;
        }
    }
}

double computePatchZNCC(
    const uint8_t* p0, int w0, int u0, int v0,
    const uint8_t* p1, int w1, int u1, int v1,
    int patchW, int patchH
) {
    const int n = patchW * patchH;
    double sum0 = 0.0, sum1 = 0.0;
    for (int dy = 0; dy < patchH; ++dy) {
        const uint8_t* r0 = p0 + (v0 + dy) * w0 + u0;
        const uint8_t* r1 = p1 + (v1 + dy) * w1 + u1;
        for (int dx = 0; dx < patchW; ++dx) {
            sum0 += r0[dx];
            sum1 += r1[dx];
        }
    }
    const double mean0 = sum0 / n;
    const double mean1 = sum1 / n;

#ifdef __APPLE__
    // Vectorized zero-mean dot product & norm via Accelerate vDSP
    float buf0[320];
    float buf1[320];
    int idx = 0;
    for (int dy = 0; dy < patchH; ++dy) {
        const uint8_t* r0 = p0 + (v0 + dy) * w0 + u0;
        const uint8_t* r1 = p1 + (v1 + dy) * w1 + u1;
        for (int dx = 0; dx < patchW; ++dx) {
            buf0[idx] = static_cast<float>(r0[dx] - mean0);
            buf1[idx] = static_cast<float>(r1[dx] - mean1);
            ++idx;
        }
    }
    float dot = 0.0f, n0 = 0.0f, n1 = 0.0f;
    vDSP_dotpr(buf0, 1, buf1, 1, &dot, static_cast<vDSP_Length>(n));
    vDSP_svesq(buf0, 1, &n0, static_cast<vDSP_Length>(n));
    vDSP_svesq(buf1, 1, &n1, static_cast<vDSP_Length>(n));
    const double denom = std::sqrt(static_cast<double>(n0) * static_cast<double>(n1));
    return (denom > 1e-6) ? (static_cast<double>(dot) / denom) : -1.0;
#else
    double dot = 0.0, n0 = 0.0, n1 = 0.0;
    for (int dy = 0; dy < patchH; ++dy) {
        const uint8_t* r0 = p0 + (v0 + dy) * w0 + u0;
        const uint8_t* r1 = p1 + (v1 + dy) * w1 + u1;
        for (int dx = 0; dx < patchW; ++dx) {
            const double a = r0[dx] - mean0;
            const double b = r1[dx] - mean1;
            dot += a * b;
            n0 += a * a;
            n1 += b * b;
        }
    }
    const double denom = std::sqrt(n0 * n1);
    return (denom > 1e-6) ? (dot / denom) : -1.0;
#endif
}

// 2-iteration Lucas-Kanade spatial-gradient sub-pixel refinement
bool refinePatchLK2(
    const uint8_t* img0, const uint8_t* img1, int w, int h,
    int u0, int v0, int patchW, int patchH,
    double& ioDu, double& ioDv
) {
    for (int iter = 0; iter < 2; ++iter) {
        double gxx = 0.0, gxy = 0.0, gyy = 0.0;
        double bx = 0.0, by = 0.0;

        for (int dy = 1; dy < patchH - 1; ++dy) {
            const int y0 = v0 + dy;
            const double y1 = static_cast<double>(y0) + ioDv;
            if (y1 < 1.0 || y1 >= h - 2) return false;

            for (int dx = 1; dx < patchW - 1; ++dx) {
                const int x0 = u0 + dx;
                const double x1 = static_cast<double>(x0) + ioDu;
                if (x1 < 1.0 || x1 >= w - 2) return false;

                const double ix = 0.5 * (static_cast<double>(img0[y0 * w + x0 + 1]) - static_cast<double>(img0[y0 * w + x0 - 1]));
                const double iy = 0.5 * (static_cast<double>(img0[(y0 + 1) * w + x0]) - static_cast<double>(img0[(y0 - 1) * w + x0]));
                const double it = sampleBilinearU8(img1, w, x1, y1) - static_cast<double>(img0[y0 * w + x0]);

                gxx += ix * ix;
                gxy += ix * iy;
                gyy += iy * iy;
                bx += ix * it;
                by += iy * it;
            }
        }

        const double det = gxx * gyy - gxy * gxy;
        if (std::fabs(det) < 1e-3) return false;

        // Solve G * delta = -b (with zero-mean residual tolerance)
        const double stepU = (-gyy * bx + gxy * by) / det;
        const double stepV = (gxy * bx - gxx * by) / det;
        if (std::hypot(stepU, stepV) > 2.0) return false;

        ioDu += stepU;
        ioDv += stepV;
    }
    return std::isfinite(ioDu) && std::isfinite(ioDv);
}

} // namespace

void TranslationalStabilizer2D::reset(const CalibrationProfile& profile, const TrajectoryResult& traj, double strength) {
    profile_ = profile;
    rawQuats_ = traj.rawFrameQuats;
    smoothQuats_ = traj.smoothFrameQuats;
    strength_ = std::min(1.0, std::max(0.0, strength));

    if (profile.height > profile.width) {
        rectW_ = 90;
        rectH_ = 160;
    } else {
        rectW_ = 160;
        rectH_ = 90;
    }

    const size_t nPix = static_cast<size_t>(rectW_ * rectH_);
    scratchDown_.assign(nPix, 0);
    prevRect_.assign(nPix, 0);
    currRect_.assign(nPix, 0);
    prevMask_.assign(nPix, 0);
    currMask_.assign(nPix, 0);
    hasPrev_ = false;

    const size_t nFrames = std::min(rawQuats_.size(), smoothQuats_.size());
    deltas_.assign(nFrames, FrameDelta{});
}

void TranslationalStabilizer2D::feedLumaFrame(size_t frameIndex, const uint8_t* fullLuma, int srcW, int srcH, int srcStride) {
    if (strength_ <= 0.001 || !fullLuma || srcW < 64 || srcH < 64 || profile_.width == 0 || profile_.height == 0) {
        return;
    }

    downsampleBoxAvg(fullLuma, srcW, srcH, srcStride, scratchDown_.data(), rectW_, rectH_);

    if (!hasPrev_) {
        prevRect_ = scratchDown_;
        prevMask_.assign(static_cast<size_t>(rectW_ * rectH_), 1);
        hasPrev_ = true;
        return;
    }

    // Pure Inter-Frame Gyro De-rotation:
    // De-rotate frame i (scratchDown_) to frame i-1's orientation using relative rotation:
    // qCorr = q_raw(t_i)^(-1) * q_raw(t_{i-1}) = q_raw[i].conjugate() * q_raw[i-1].
    // If camera purely rotates (no 3D translation), frame i after de-rotation matches frame i-1 (dx=0, dy=0)!
    const size_t nTraj = rawQuats_.size();
    Quat qCorr = Quat::identity();
    if (nTraj > 0 && frameIndex > 0) {
        const size_t qCur = std::min(frameIndex, nTraj - 1);
        const size_t qPrev = std::min(frameIndex - 1, nTraj - 1);
        qCorr = (rawQuats_[qCur].conjugate() * rawQuats_[qPrev]).normalized();
    }

    const double scaleX = static_cast<double>(rectW_) / static_cast<double>(profile_.width);
    const double scaleY = static_cast<double>(rectH_) / static_cast<double>(profile_.height);
    const double scaleF = static_cast<double>(std::max(rectW_, rectH_)) / static_cast<double>(std::max(profile_.width, profile_.height));
    rectifyUnzoomedFrame(
        scratchDown_.data(), rectW_, rectH_, qCorr,
        profile_.fx * scaleX * (scaleF / scaleX),
        profile_.fy * scaleF,
        profile_.cx * scaleX,
        profile_.cy * scaleY,
        currRect_.data(), currMask_.data()
    );

    if (frameIndex < deltas_.size()) {
        const int searchRad = 6;
        const int gridCols = (rectW_ >= rectH_) ? 6 : 4;
        const int gridRows = (rectW_ >= rectH_) ? 4 : 6;
        const int patchW = (rectW_ >= rectH_) ? 20 : 16;
        const int patchH = (rectW_ >= rectH_) ? 16 : 20;
        const int usableW = rectW_ - 2 * searchRad - patchW;
        const int usableH = rectH_ - 2 * searchRad - patchH;

        struct Obs2D {
            double du, dv;
        };
        std::vector<Obs2D> obs;
        obs.reserve(static_cast<size_t>(gridCols * gridRows));

        for (int gy = 0; gy < gridRows; ++gy) {
            for (int gx = 0; gx < gridCols; ++gx) {
                const int u0 = searchRad + (usableW * gx) / std::max(1, gridCols - 1);
                const int v0 = searchRad + (usableH * gy) / std::max(1, gridRows - 1);

                // Check 100% validMask across patch in prev AND patch+-searchRad in curr + structural gradient
                bool allValid = true;
                double gradSum = 0.0;
                for (int dy = 0; dy < patchH && allValid; ++dy) {
                    const uint8_t* m0 = prevMask_.data() + (v0 + dy) * rectW_ + u0;
                    const uint8_t* p0 = prevRect_.data() + (v0 + dy) * rectW_ + u0;
                    for (int dx = 0; dx < patchW; ++dx) {
                        if (m0[dx] == 0) {
                            allValid = false;
                            break;
                        }
                        if (dx + 1 < patchW && dy + 1 < patchH) {
                            const int gxV = std::abs(static_cast<int>(p0[dx + 1]) - static_cast<int>(p0[dx]));
                            const int gyV = std::abs(static_cast<int>(prevRect_[(v0 + dy + 1) * rectW_ + u0 + dx]) - static_cast<int>(p0[dx]));
                            gradSum += std::min(gxV, gyV);
                        }
                    }
                }
                if (!allValid || (gradSum / (patchW * patchH)) < 2.2) {
                    continue;
                }

                for (int dy = -searchRad; dy < patchH + searchRad && allValid; ++dy) {
                    const uint8_t* m1 = currMask_.data() + (v0 + dy) * rectW_ + u0;
                    for (int dx = -searchRad; dx < patchW + searchRad; ++dx) {
                        if (m1[dx] == 0) {
                            allValid = false;
                            break;
                        }
                    }
                }
                if (!allValid) {
                    continue;
                }

                // Coarse integer ZNCC search in [-6, +6]
                int bestDu = 0, bestDv = 0;
                double bestZncc = -1.0;
                for (int dv = -searchRad; dv <= searchRad; ++dv) {
                    for (int du = -searchRad; du <= searchRad; ++du) {
                        const double zncc = computePatchZNCC(
                            prevRect_.data(), rectW_, u0, v0,
                            currRect_.data(), rectW_, u0 + du, v0 + dv,
                            patchW, patchH
                        );
                        if (zncc > bestZncc) {
                            bestZncc = zncc;
                            bestDu = du;
                            bestDv = dv;
                        }
                    }
                }
                if (bestZncc < 0.65) {
                    continue;
                }

                // Forward 2-iteration LK sub-pixel refinement
                double fDu = static_cast<double>(bestDu);
                double fDv = static_cast<double>(bestDv);
                if (!refinePatchLK2(prevRect_.data(), currRect_.data(), rectW_, rectH_, u0, v0, patchW, patchH, fDu, fDv)) {
                    continue;
                }

                // Forward-Backward consistency check (< 0.35 px at 160x90)
                const int u1 = static_cast<int>(std::round(u0 + fDu));
                const int v1 = static_cast<int>(std::round(v0 + fDv));
                if (u1 < 1 || u1 + patchW >= rectW_ - 1 || v1 < 1 || v1 + patchH >= rectH_ - 1) {
                    continue;
                }
                double bDu = -fDu;
                double bDv = -fDv;
                if (!refinePatchLK2(currRect_.data(), prevRect_.data(), rectW_, rectH_, u1, v1, patchW, patchH, bDu, bDv)) {
                    continue;
                }
                const double fbErr = std::hypot(fDu + bDu, fDv + bDv);
                if (fbErr > 0.35) {
                    continue;
                }

                obs.push_back({fDu, fDv});
            }
        }

        if (obs.size() >= 3) {
            // RANSAC 2D consensus (inlier threshold = 0.45 px) + IRLS inlier mean
            int bestInliers = -1;
            double bestErrSum = 1e18;
            double bestX = 0.0, bestY = 0.0;
            constexpr double kThresh = 0.45;

            for (size_t i = 0; i < obs.size(); ++i) {
                int inliers = 0;
                double errSum = 0.0;
                for (size_t j = 0; j < obs.size(); ++j) {
                    const double e = std::hypot(obs[j].du - obs[i].du, obs[j].dv - obs[i].dv);
                    if (e <= kThresh) {
                        ++inliers;
                        errSum += e;
                    }
                }
                if (inliers > bestInliers || (inliers == bestInliers && errSum < bestErrSum)) {
                    bestInliers = inliers;
                    bestErrSum = errSum;
                    bestX = obs[i].du;
                    bestY = obs[i].dv;
                }
            }

            if (bestInliers >= 3) {
                double sumU = 0.0, sumV = 0.0, sumW = 0.0;
                for (const auto& o : obs) {
                    const double e = std::hypot(o.du - bestX, o.dv - bestY);
                    if (e <= kThresh) {
                        const double w = 1.0 / (1.0 + (e / 0.25) * (e / 0.25));
                        sumU += w * o.du;
                        sumV += w * o.dv;
                        sumW += w;
                    }
                }
                if (sumW > 1e-6) {
                    deltas_[frameIndex] = {sumU / sumW, sumV / sumW, 1.0};
                }
            }
        }
    }

    // Save current raw downsampled frame for next frame's pure inter-frame de-rotation
    prevRect_ = scratchDown_;
    prevMask_.assign(static_cast<size_t>(rectW_ * rectH_), 1);
    hasPrev_ = true;
}

Translational2DResult TranslationalStabilizer2D::finalize(double nominalFps) const {
    Translational2DResult res;
    if (strength_ <= 0.001) {
        return res; // Opt-in: bypass completely when strength is 0
    }
    const size_t n = deltas_.size();
    if (n < 3 || profile_.width == 0 || profile_.height == 0) {
        return res;
    }

    const double scaleToFullX = static_cast<double>(profile_.width) / static_cast<double>(rectW_);
    const double scaleToFullY = static_cast<double>(profile_.height) / static_cast<double>(rectH_);
    const double dt = (nominalFps > 1.0) ? (1.0 / nominalFps) : (1.0 / 30.0);

    std::vector<double> posX(n, 0.0);
    std::vector<double> posY(n, 0.0);
    size_t validCount = 0;

    for (size_t i = 1; i < n; ++i) {
        if (deltas_[i].confidence > 0.5) {
            posX[i] = posX[i - 1] + deltas_[i].dx160 * scaleToFullX;
            posY[i] = posY[i - 1] + deltas_[i].dy160 * scaleToFullY;
            ++validCount;
        } else {
            // Gentle decay toward 0 during low-confidence whip-pan spans to avoid static offset lock
            const double decay = std::exp(-dt / 0.35);
            posX[i] = posX[i - 1] * decay;
            posY[i] = posY[i - 1] * decay;
        }
    }

    // Zero-Phase Narrow Band-Pass Filter (1.5 Hz - 4.5 Hz):
    // LP (tau = 0.032s ~ 5.0 Hz cutoff) suppresses high-frequency sub-pixel quantization noise.
    // HP (subtracting baseline tau = 0.133s ~ 1.2 Hz cutoff) eliminates low-frequency drift.
    auto zeroPhaseLPF = [&](const std::vector<double>& in, double tau) -> std::vector<double> {
        const double alpha = std::exp(-dt / std::max(tau, 1e-4));
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

    const std::vector<double> lpX = zeroPhaseLPF(posX, 0.012);
    const std::vector<double> lpY = zeroPhaseLPF(posY, 0.012);
    const std::vector<double> baseDriftX = zeroPhaseLPF(posX, 0.160);
    const std::vector<double> baseDriftY = zeroPhaseLPF(posY, 0.160);

    res.shiftXBase.resize(n, 0.0);
    res.shiftYBase.resize(n, 0.0);
    double maxAmp = 0.0, sumSq = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double sx = lpX[i] - baseDriftX[i];
        double sy = lpY[i] - baseDriftY[i];

        // Deadzone: eliminate micro-jitter (< 0.25 px on 1080p)
        const double mag = std::hypot(sx, sy);
        if (mag < 0.25) {
            sx = 0.0;
            sy = 0.0;
        } else {
            // Hard clamp to +-24 px maximum
            constexpr double kMaxClamp = 24.0;
            if (mag > kMaxClamp) {
                const double sc = kMaxClamp / mag;
                sx *= sc;
                sy *= sc;
            }
        }

        sx *= strength_;
        sy *= strength_;

        res.shiftXBase[i] = sx;
        res.shiftYBase[i] = sy;
        const double amp = std::hypot(sx, sy);
        maxAmp = std::max(maxAmp, amp);
        sumSq += amp * amp;
    }

    res.maxShiftPx = maxAmp;
    res.rmsShiftPx = std::sqrt(sumSq / static_cast<double>(n));
    res.validFrameCount = validCount;
    res.valid = (validCount > 0);
    return res;
}

} // namespace gf
