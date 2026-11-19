#include "CalibrationProfile.hpp"
#include "FrameWarpCalculator.hpp"
#include "Quaternion.hpp"
#include "TrajectoryEngine.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

double sincD(double x) {
    if (std::fabs(x) < 1e-6) return 1.0;
    const double pix = kPi * x;
    return std::sin(pix) / pix;
}

double lanczos4Weight(double x) {
    const double ax = std::fabs(x);
    if (ax >= 4.0) return 0.0;
    return sincD(x) * sincD(x * 0.25);
}

double smoothstepD(double e0, double e1, double x) {
    const double t = std::min(1.0, std::max(0.0, (x - e0) / (e1 - e0)));
    return t * t * (3.0 - 2.0 * t);
}

double evalRichPattern(double x, double y, int w, int h) {
    const double cx = 0.5 * w;
    const double cy = 0.5 * h;
    const double dx = x - cx;
    const double dy = y - cy;
    const double r = std::hypot(dx, dy);
    const double theta = std::atan2(dy, dx);

    // Smooth radial Siemens-like star (band-limited) + multi-angle ramps & smooth gradients
    const double starEnv = smoothstepD(25.0, 55.0, r) * (1.0 - smoothstepD(120.0, 160.0, r));
    const double star = 0.22 * starEnv * std::sin(14.0 * theta);
    const double ramp = 0.28 * (0.5 + 0.5 * std::sin(0.032 * x + 0.019 * y));
    const double waves = 0.16 * std::cos(0.055 * x - 0.041 * y) + 0.12 * std::sin(0.025 * r);
    const double val = 0.48 + star + (ramp - 0.14) + waves;
    return std::min(1.0, std::max(0.0, val));
}

double computePSNR(const std::vector<double>& a, const std::vector<double>& b) {
    assert(a.size() == b.size() && !a.empty());
    double mse = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        const double d = a[i] - b[i];
        mse += d * d;
    }
    mse /= static_cast<double>(a.size());
    if (mse <= 1e-12) return 99.0;
    return 10.0 * std::log10(1.0 / mse);
}

double computeSSIM(const std::vector<double>& a, const std::vector<double>& b, int w, int h) {
    assert(a.size() == static_cast<size_t>(w * h));
    assert(b.size() == static_cast<size_t>(w * h));
    constexpr double C1 = 0.01 * 0.01;
    constexpr double C2 = 0.03 * 0.03;

    double ssimSum = 0.0;
    int count = 0;
    const int win = 4;
    for (int y = win; y < h - win; y += 2) {
        for (int x = win; x < w - win; x += 2) {
            double muA = 0.0, muB = 0.0;
            int n = 0;
            for (int dy = -win; dy <= win; ++dy) {
                for (int dx = -win; dx <= win; ++dx) {
                    const size_t idx = static_cast<size_t>((y + dy) * w + (x + dx));
                    muA += a[idx];
                    muB += b[idx];
                    ++n;
                }
            }
            muA /= n;
            muB /= n;

            double varA = 0.0, varB = 0.0, cov = 0.0;
            for (int dy = -win; dy <= win; ++dy) {
                for (int dx = -win; dx <= win; ++dx) {
                    const size_t idx = static_cast<size_t>((y + dy) * w + (x + dx));
                    const double da = a[idx] - muA;
                    const double db = b[idx] - muB;
                    varA += da * da;
                    varB += db * db;
                    cov += da * db;
                }
            }
            varA /= (n - 1);
            varB /= (n - 1);
            cov /= (n - 1);

            const double num = (2.0 * muA * muB + C1) * (2.0 * cov + C2);
            const double den = (muA * muA + muB * muB + C1) * (varA + varB + C2);
            ssimSum += num / den;
            ++count;
        }
    }
    return (count > 0) ? (ssimSum / count) : 1.0;
}

double sampleLanczos4CasCPU(
    const std::vector<double>& img, int w, int h,
    double sx, double sy, double sharpenStrength, double currentZoom
) {
    const int maxX = w - 1;
    const int maxY = h - 1;
    const int ix = static_cast<int>(std::floor(sx));
    const int iy = static_cast<int>(std::floor(sy));
    const double fx = sx - ix;
    const double fy = sy - iy;

    double wx[8], wy[8];
    for (int k = 0; k < 8; ++k) {
        const double offset = static_cast<double>(k - 3);
        wx[k] = lanczos4Weight(fx - offset);
        wy[k] = lanczos4Weight(fy - offset);
    }

    auto readAt = [&](int x, int y) -> double {
        const int cx = std::min(maxX, std::max(0, x));
        const int cy = std::min(maxY, std::max(0, y));
        return img[static_cast<size_t>(cy * w + cx)];
    };

    const double c00 = readAt(ix, iy);
    const double c10 = readAt(ix + 1, iy);
    const double c01 = readAt(ix, iy + 1);
    const double c11 = readAt(ix + 1, iy + 1);
    const double min2x2 = std::min(std::min(c00, c10), std::min(c01, c11));
    const double max2x2 = std::max(std::max(c00, c10), std::max(c01, c11));

    double sum = 0.0, wSum = 0.0;
    for (int dy = -3; dy <= 4; ++dy) {
        const double wRow = wy[dy + 3];
        for (int dx = -3; dx <= 4; ++dx) {
            const double weight = wx[dx + 3] * wRow;
            sum += readAt(ix + dx, iy + dy) * weight;
            wSum += weight;
        }
    }

    const double rawY = sum / std::max(wSum, 1e-6);
    const double clampedY = std::min(max2x2 + 0.025, std::max(min2x2 - 0.025, rawY));
    const double relaxedY = rawY * 0.25 + clampedY * 0.75;

    if (sharpenStrength <= 1e-4) {
        return std::min(1.0, std::max(0.0, relaxedY));
    }

    const double localContrast = max2x2 - min2x2;
    const double flatNoiseGate = smoothstepD(0.015, 0.05, localContrast);
    const double haloGuard = std::min(1.0, std::max(0.0, 1.0 - max2x2));
    const double zoomBoost = std::min(1.65, std::max(1.0, 1.0 + 0.85 * (currentZoom - 1.0)));
    const double avg2x2 = 0.25 * (c00 + c10 + c01 + c11);
    const double detail = relaxedY - avg2x2;
    const double casWeight = sharpenStrength * zoomBoost * flatNoiseGate * (0.35 + 0.65 * haloGuard);
    return std::min(1.0, std::max(0.0, relaxedY + casWeight * detail));
}

double sampleBilinearCPU(const std::vector<double>& img, int w, int h, double sx, double sy) {
    const int maxX = w - 1;
    const int maxY = h - 1;
    const int x0 = std::min(maxX, std::max(0, static_cast<int>(std::floor(sx))));
    const int y0 = std::min(maxY, std::max(0, static_cast<int>(std::floor(sy))));
    const int x1 = std::min(x0 + 1, maxX);
    const int y1 = std::min(y0 + 1, maxY);
    const double fx = std::min(1.0, std::max(0.0, sx - x0));
    const double fy = std::min(1.0, std::max(0.0, sy - y0));
    const double v00 = img[static_cast<size_t>(y0 * w + x0)];
    const double v10 = img[static_cast<size_t>(y0 * w + x1)];
    const double v01 = img[static_cast<size_t>(y1 * w + x0)];
    const double v11 = img[static_cast<size_t>(y1 * w + x1)];
    const double top = v00 * (1.0 - fx) + v10 * fx;
    const double bot = v01 * (1.0 - fx) + v11 * fx;
    return top * (1.0 - fy) + bot * fy;
}

void testSinglePassResamplingFidelity(double& outRotPsnr, double& outRotSsim) {
    const int w = 640;
    const int h = 360;
    const double fx = 553.0, fy = 553.0, cx = 320.0, cy = 180.0;

    std::vector<double> srcImg(static_cast<size_t>(w * h));
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            srcImg[static_cast<size_t>(y * w + x)] = evalRichPattern(x + 0.5, y + 0.5, w, h);
        }
    }

    // 1A: Identity transform (Zoom=1.0, Rot=0)
    std::vector<double> idOut(static_cast<size_t>(w * h));
    float H_id[9];
    gf::FrameWarpCalculator::computeInverseHomography(gf::Quat::identity(), fx, fy, cx, cy, 1.0, H_id);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const double px = x + 0.5;
            const double py = y + 0.5;
            const double ww = H_id[6] * px + H_id[7] * py + H_id[8];
            const double sx = (H_id[0] * px + H_id[1] * py + H_id[2]) / ww - 0.5;
            const double sy = (H_id[3] * px + H_id[4] * py + H_id[5]) / ww - 0.5;
            idOut[static_cast<size_t>(y * w + x)] = sampleLanczos4CasCPU(srcImg, w, h, sx, sy, 0.45, 1.0);
        }
    }
    const double idPsnr = computePSNR(srcImg, idOut);
    const double idSsim = computeSSIM(srcImg, idOut, w, h);
    assert(idPsnr > 55.0);
    assert(idSsim > 0.999);

    // 1B: +12.5 deg rotation vs analytical ground-truth
    const gf::Quat qRot = gf::Quat::fromAxisAngle({0.0, 0.0, 1.0}, 12.5 * (kPi / 180.0));
    float H_rot[9];
    gf::FrameWarpCalculator::computeInverseHomography(qRot, fx, fy, cx, cy, 1.25, H_rot);

    std::vector<double> rotWarp;
    std::vector<double> rotTruth;
    const int cropW = 480, cropH = 260;
    rotWarp.reserve(static_cast<size_t>(cropW * cropH));
    rotTruth.reserve(static_cast<size_t>(cropW * cropH));

    for (int y = 50; y < 310; ++y) {
        for (int x = 80; x < 560; ++x) {
            const double px = x + 0.5;
            const double py = y + 0.5;
            const double ww = H_rot[6] * px + H_rot[7] * py + H_rot[8];
            const double sx = (H_rot[0] * px + H_rot[1] * py + H_rot[2]) / ww - 0.5;
            const double sy = (H_rot[3] * px + H_rot[4] * py + H_rot[5]) / ww - 0.5;
            if (sx >= 8.0 && sx < w - 9 && sy >= 8.0 && sy < h - 9) {
                rotWarp.push_back(sampleLanczos4CasCPU(srcImg, w, h, sx, sy, 0.35, 1.25));
                rotTruth.push_back(evalRichPattern(sx + 0.5, sy + 0.5, w, h));
            }
        }
    }
    outRotPsnr = computePSNR(rotTruth, rotWarp);
    outRotSsim = computeSSIM(rotTruth, rotWarp, static_cast<int>(rotTruth.size() / 260), 260);
    assert(outRotPsnr > 38.5);
    assert(outRotSsim > 0.980);

    std::printf("Quality Test 1 (Identity PSNR=%.2f dB, SSIM=%.5f | +12.5deg Single-Pass PSNR=%.2f dB, SSIM=%.4f) PASSED.\n",
                idPsnr, idSsim, outRotPsnr, outRotSsim);
}

void testTemporalFullFrameEdgeFill() {
    const int w = 640, h = 360;
    const double fx = 553.0, fy = 553.0, cx = 320.0, cy = 180.0;

    // Frame 0 rotated -5.5 deg yaw (sees extra left scene), Frame 2 rotated +5.5 deg yaw (missing left edge when stabilized to 0 deg!)
    const gf::Quat qRaw0 = gf::Quat::fromAxisAngle({0.0, 1.0, 0.0}, -5.5 * (kPi / 180.0));
    const gf::Quat qRaw2 = gf::Quat::fromAxisAngle({0.0, 1.0, 0.0}, +5.5 * (kPi / 180.0));
    const gf::Quat qTarget = gf::Quat::identity();

    auto renderSynthCamera = [&](const gf::Quat& qCam) -> std::vector<double> {
        std::vector<double> img(static_cast<size_t>(w * h));
        double R[9];
        gf::FrameWarpCalculator::computeRotationMatrix3x3(qCam, R);
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                const double xn = (x + 0.5 - cx) / fx;
                const double yn = (y + 0.5 - cy) / fy;
                const double wx = R[0] * xn + R[1] * yn + R[2];
                const double wy = R[3] * xn + R[4] * yn + R[5];
                const double wz = R[6] * xn + R[7] * yn + R[8];
                const double uW = fx * (wx / wz) + cx;
                const double vW = fy * (wy / wz) + cy;
                img[static_cast<size_t>(y * w + x)] = evalRichPattern(uW, vW, w, h);
            }
        }
        return img;
    };

    const std::vector<double> frame0 = renderSynthCamera(qRaw0);
    const std::vector<double> frame2 = renderSynthCamera(qRaw2);
    const std::vector<double> truth0 = renderSynthCamera(qTarget);

    // Compute H_cur (frame 2 -> target) and H_hist0 (frame 0 -> target) at Zoom = 1.00x!
    float H_cur[9], H_hist[9];
    gf::FrameWarpCalculator::computeInverseHomography((qRaw2.conjugate() * qTarget).normalized(), fx, fy, cx, cy, 1.0, H_cur);
    gf::FrameWarpCalculator::computeInverseHomography((qRaw0.conjugate() * qTarget).normalized(), fx, fy, cx, cy, 1.0, H_hist);

    std::vector<double> filledBorderTruth;
    std::vector<double> filledBorderRecon;
    int blackPixels = 0;

    for (int y = 20; y < h - 20; ++y) {
        for (int x = 10; x < w - 10; ++x) {
            const double px = x + 0.5, py = y + 0.5;
            const double wc = H_cur[6] * px + H_cur[7] * py + H_cur[8];
            const double sx = (H_cur[0] * px + H_cur[1] * py + H_cur[2]) / wc - 0.5;
            const double sy = (H_cur[3] * px + H_cur[4] * py + H_cur[5]) / wc - 0.5;

            if (sx < 0.0 || sx > w - 1 || sy < 0.0 || sy > h - 1) {
                const double wh = H_hist[6] * px + H_hist[7] * py + H_hist[8];
                const double hsx = (H_hist[0] * px + H_hist[1] * py + H_hist[2]) / wh - 0.5;
                const double hsy = (H_hist[3] * px + H_hist[4] * py + H_hist[5]) / wh - 0.5;
                if (hsx >= 2.0 && hsx <= w - 3 && hsy >= 2.0 && hsy <= h - 3) {
                    const double hVal = sampleBilinearCPU(frame0, w, h, hsx, hsy);
                    filledBorderRecon.push_back(hVal);
                    filledBorderTruth.push_back(truth0[static_cast<size_t>(y * w + x)]);
                } else {
                    ++blackPixels;
                }
            }
        }
    }

    assert(!filledBorderRecon.empty());
    assert(blackPixels == 0);
    const double borderPsnr = computePSNR(filledBorderTruth, filledBorderRecon);
    assert(borderPsnr > 36.0);

    std::printf("Quality Test 2 (Temporal Edge Fill @ Zoom=1.00x -> %zu border px filled, 0 black px, Border PSNR=%.2f dB) PASSED.\n",
                filledBorderRecon.size(), borderPsnr);
}

void testFinalBenchmarkScorecard(double rotPsnr, double rotSsim) {
    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.width = 1920;
    hdr.height = 1080;
    hdr.nominal_fps = 30.0;

    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    for (int i = 0; i < 270; ++i) {
        const double t = i / 90.0;
        const double dPitch = (5.5 * kPi / 180.0) * (2.0 * kPi * 4.0) * std::cos(2.0 * kPi * 4.0 * t);
        const double dYaw = (4.5 * kPi / 180.0) * (2.0 * kPi * 3.5) * std::cos(2.0 * kPi * 3.5 * t);
        gyro.push_back({t, -dYaw, -dPitch, 0.0});
        accel.push_back({t, 0.0f, -1.0f, 0.0f});
    }
    std::vector<gf::GFFrameRecord> frames;
    for (int f = 0; f < 90; ++f) {
        frames.push_back({static_cast<uint32_t>(f), f / 30.0, 0.0, 100.0f, 0.7f, 0.0152f});
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel);
    gf::StabilizerConfig cfg;
    cfg.pitchSmoothness = 0.85;
    cfg.yawSmoothness = 0.85;
    cfg.rollSmoothness = 0.92;
    cfg.horizonLockEnabled = true;

    const auto traj = engine.solve(frames, cfg);
    assert(traj.valid);
    const auto prof = gf::CalibrationProfile::defaultIPhone5s1080p30();
    const auto warp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        engine, traj, prof, 30.0, 0.0, true, true
    );
    const double meanCropPct = (1.0 - 1.0 / (warp.avgZoom * warp.avgZoom)) * 100.0;
    const double maxCropPct = (1.0 - 1.0 / (warp.maxZoom * warp.maxZoom)) * 100.0;

    std::printf(
        "\n=== FINAL CINEMA MASTER v1.0.0 BENCHMARK SCORECARD ===\n"
        "  Stability Score:            %.2f%%\n"
        "  Mean / Max Cropping Ratio:  %.1f%% / %.1f%% (Zoom avg=%.2fx, max=%.2fx)\n"
        "  Residual Horizon Tilt Err:  %.3f deg\n"
        "  Single-Pass Resampling:     PSNR=%.2f dB | SSIM=%.4f\n"
        "=======================================================\n",
        traj.stabilityImprovementPct,
        meanCropPct, maxCropPct, warp.avgZoom, warp.maxZoom,
        traj.maxOutputRollErrorDeg,
        rotPsnr, rotSsim
    );
}

} // namespace

int main() {
    double rotPsnr = 0.0, rotSsim = 0.0;
    testSinglePassResamplingFidelity(rotPsnr, rotSsim);
    testTemporalFullFrameEdgeFill();
    testFinalBenchmarkScorecard(rotPsnr, rotSsim);
    return 0;
}
