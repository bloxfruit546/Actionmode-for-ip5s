#include "CalibrationProfile.hpp"
#include "FrameWarpCalculator.hpp"
#include "QualityMetrics.hpp"
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
    double sx, double sy, double sharpenStrength, double currentZoom,
    bool legacyCas = false
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

    const double haloGuard = std::min(1.0, std::max(0.0, 1.0 - max2x2));
    const double avg2x2 = 0.25 * (c00 + c10 + c01 + c11);
    const double detail = relaxedY - avg2x2;

    if (legacyCas) {
        const double localContrast = max2x2 - min2x2;
        const double flatNoiseGate = smoothstepD(0.015, 0.05, localContrast);
        const double zoomBoost = std::min(2.05, std::max(1.0, 1.0 + 1.15 * (currentZoom - 1.0)));
        const double casWeight = sharpenStrength * zoomBoost * flatNoiseGate * (0.35 + 0.65 * haloGuard);
        return std::min(1.0, std::max(0.0, relaxedY + casWeight * detail));
    }

    const double gradX = 0.5 * std::fabs((c10 + c11) - (c00 + c01));
    const double gradY = 0.5 * std::fabs((c01 + c11) - (c00 + c10));
    const double coherentGrad = std::max(gradX, gradY);
    const double flatNoiseGate = smoothstepD(0.025, 0.085, coherentGrad);
    const double zoomBoost = std::min(1.60, std::max(1.0, 1.0 + 0.70 * (currentZoom - 1.0)));
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

void testQualityMetricsAndVideoAnalyzer() {
    // 1. Compare clean pan vs high-freq shaky pan vs kinked clamp path
    const int n = 150;
    const double fps = 30.0;
    const double dt = 1.0 / fps;
    std::vector<gf::Vec3> smoothOmega;
    std::vector<gf::Vec3> shakyOmega;
    std::vector<gf::Vec3> kinkedOmega;
    std::vector<double> zoomUsed(static_cast<size_t>(n), 1.32);

    for (int i = 0; i + 1 < n; ++i) {
        const double t = i * dt;
        const double pan = 0.4 * std::sin(kPi * t / 5.0);
        smoothOmega.emplace_back(0.0, pan, 0.0);
        shakyOmega.emplace_back(
            0.35 * std::sin(2.0 * kPi * 5.0 * t),
            pan + 0.30 * std::cos(2.0 * kPi * 4.5 * t),
            0.18 * std::sin(2.0 * kPi * 6.0 * t)
        );
        double kink = 0.0;
        if (i == 45) kink = 0.35;
        if (i == 46) kink = -0.35;
        if (i == 95) kink = -0.40;
        if (i == 96) kink = 0.40;
        kinkedOmega.emplace_back(kink, pan, 0.0);
    }

    const auto rSmooth = gf::QualityMetrics::evaluateOmegaSequence(smoothOmega, fps, &zoomUsed, 0.1);
    const auto rShaky = gf::QualityMetrics::evaluateOmegaSequence(shakyOmega, fps, &zoomUsed, 0.1);
    const auto rKink = gf::QualityMetrics::evaluateOmegaSequence(kinkedOmega, fps, &zoomUsed, 0.1);

    assert(rSmooth.highFreqOmegaEnergyAbove2Hz < 0.01);
    assert(rShaky.highFreqOmegaEnergyAbove2Hz > 0.10);
    assert(rSmooth.liuStabilityScore > 0.95);
    assert(rShaky.liuStabilityScore < rSmooth.liuStabilityScore);
    assert(rSmooth.jerkRms < rShaky.jerkRms);
    assert(rKink.accelDiscontinuityCount >= 2);
    assert(std::fabs(rSmooth.meanCroppingRatio - (1.0 / 1.32)) < 1e-6);

    // 2. Streaming VideoQualityAnalyzer determinism check using VirtualGyroEngine
    const int w = 320, h = 180;
    gf::VirtualGyroConfig vgCfg;
    vgCfg.videoWidth = w;
    vgCfg.videoHeight = h;
    vgCfg.nominalFps = 30.0;
    vgCfg.lensFx = 300.0;
    vgCfg.lensFy = 300.0;
    vgCfg.lensCx = 160.0;
    vgCfg.lensCy = 90.0;

    auto runStream = [&]() -> std::string {
        gf::VideoQualityAnalyzer vqa(vgCfg);
        std::vector<uint8_t> img(static_cast<size_t>(w * h));
        for (int f = 0; f < 30; ++f) {
            const double shiftX = 2.5 * std::sin(2.0 * kPi * 4.0 * (f / 30.0));
            for (int y = 0; y < h; ++y) {
                for (int x = 0; x < w; ++x) {
                    const double v = evalRichPattern(x + 0.5 - shiftX, y + 0.5, w, h) * 255.0;
                    img[static_cast<size_t>(y * w + x)] = static_cast<uint8_t>(std::min(255.0, std::max(0.0, v)));
                }
            }
            assert(vqa.pushRenderedLumaFrame(img.data(), w, h, f / 30.0));
        }
        const auto rep = vqa.finalize(nullptr, 0.0);
        assert(rep.hasVisualMetrics);
        assert(rep.highFreqOmegaEnergyAbove2Hz > 1e-4);
        assert(rep.meanInlierRatio > 0.70);
        return gf::QualityMetrics::toJson(rep, "synthetic_stream");
    };
    const std::string j1 = runStream();
    const std::string j2 = runStream();
    assert(j1 == j2);

    std::printf(
        "QualityMetrics Test PASSED: Smooth(E>2Hz=%.4f, Liu=%.4f, Jerk=%.2f) vs Shaky(E>2Hz=%.4f, Liu=%.4f, Jerk=%.2f) | Kinks=%d\n",
        rSmooth.highFreqOmegaEnergyAbove2Hz, rSmooth.liuStabilityScore, rSmooth.jerkRms,
        rShaky.highFreqOmegaEnergyAbove2Hz, rShaky.liuStabilityScore, rShaky.jerkRms,
        rKink.accelDiscontinuityCount
    );
}

double lanczos2Weight(double x) {
    const double ax = std::fabs(x);
    if (ax >= 2.0) return 0.0;
    return sincD(x) * sincD(x * 0.5);
}

double sampleLanczos2ChromaCPU(const std::vector<double>& img, int cw, int ch, double sx, double sy) {
    const int maxX = cw - 1;
    const int maxY = ch - 1;
    const int ix = static_cast<int>(std::floor(sx));
    const int iy = static_cast<int>(std::floor(sy));
    const double fx = sx - ix;
    const double fy = sy - iy;

    double wx[4], wy[4];
    for (int k = 0; k < 4; ++k) {
        const double offset = static_cast<double>(k - 1);
        wx[k] = lanczos2Weight(fx - offset);
        wy[k] = lanczos2Weight(fy - offset);
    }

    double sum = 0.0, wSum = 0.0;
    for (int dy = -1; dy <= 2; ++dy) {
        const int py = std::min(maxY, std::max(0, iy + dy));
        const double wRow = wy[dy + 1];
        for (int dx = -1; dx <= 2; ++dx) {
            const int px = std::min(maxX, std::max(0, ix + dx));
            const double weight = wx[dx + 1] * wRow;
            sum += img[static_cast<size_t>(py * cw + px)] * weight;
            wSum += weight;
        }
    }
    return std::min(1.0, std::max(0.0, sum / std::max(wSum, 1e-6)));
}

void testChroma420SitingCheckerboard() {
    const int w = 256, h = 256;
    const int cw = w / 2, ch = h / 2;
    const double fx = 240.0, fy = 240.0, cx = 128.0, cy = 128.0;

    // Band-limited smooth checkerboard (cell = 32 luma px) aligned identically in luma & chroma
    auto evalChecker = [](double px, double py) -> double {
        const double sx = std::sin(kPi * px / 32.0);
        const double sy = std::sin(kPi * py / 32.0);
        const double v = 0.5 + 0.42 * sx * sy;
        return std::min(1.0, std::max(0.0, v));
    };

    auto runCase = [&](double trueOffX, double trueOffY, double shaderOffX, double shaderOffY,
                       double zoom, double rotDeg) -> std::pair<double, double> {
        std::vector<double> srcY(static_cast<size_t>(w * h));
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                srcY[static_cast<size_t>(y * w + x)] = evalChecker(x + 0.5, y + 0.5);
            }
        }
        std::vector<double> srcC(static_cast<size_t>(cw * ch));
        for (int cyIdx = 0; cyIdx < ch; ++cyIdx) {
            for (int cxIdx = 0; cxIdx < cw; ++cxIdx) {
                srcC[static_cast<size_t>(cyIdx * cw + cxIdx)] =
                    evalChecker(2.0 * cxIdx + trueOffX, 2.0 * cyIdx + trueOffY);
            }
        }

        float H[9];
        const gf::Quat qRot = gf::Quat::fromAxisAngle({0.0, 0.0, 1.0}, rotDeg * (kPi / 180.0));
        gf::FrameWarpCalculator::computeInverseHomography(qRot, fx, fy, cx, cy, zoom, H);

        std::vector<double> outY(static_cast<size_t>(w * h));
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                const double dx = x + 0.5, dy = y + 0.5;
                const double ww = H[6] * dx + H[7] * dy + H[8];
                const double sx = (H[0] * dx + H[1] * dy + H[2]) / ww - 0.5;
                const double sy = (H[3] * dx + H[4] * dy + H[5]) / ww - 0.5;
                outY[static_cast<size_t>(y * w + x)] = sampleLanczos4CasCPU(srcY, w, h, sx, sy, 0.0, zoom);
            }
        }

        std::vector<double> outC(static_cast<size_t>(cw * ch));
        for (int gy = 0; gy < ch; ++gy) {
            for (int gx = 0; gx < cw; ++gx) {
                const double lx = 2.0 * gx + shaderOffX;
                const double ly = 2.0 * gy + shaderOffY;
                const double ww = H[6] * lx + H[7] * ly + H[8];
                const double sx = ((H[0] * lx + H[1] * ly + H[2]) / ww - shaderOffX) * 0.5;
                const double sy = ((H[3] * lx + H[4] * ly + H[5]) / ww - shaderOffY) * 0.5;
                outC[static_cast<size_t>(gy * cw + gx)] = sampleLanczos2ChromaCPU(srcC, cw, ch, sx, sy);
            }
        }

        // Measure horizontal & vertical 50%-crossing edge position error in output luma pixels
        // along row y=144 (vertical transition near x=128) and col x=144 (horizontal transition near y=128)
        auto findCross = [](const std::vector<double>& line) -> double {
            for (size_t i = 1; i < line.size(); ++i) {
                if ((line[i - 1] - 0.5) * (line[i] - 0.5) <= 0.0 && std::fabs(line[i] - line[i - 1]) > 0.01) {
                    const double t = (0.5 - line[i - 1]) / (line[i] - line[i - 1]);
                    return static_cast<double>(i - 1) + t;
                }
            }
            return -1.0;
        };

        std::vector<double> lineY_h, lineC_h;
        for (int x = 96; x < 160; ++x) {
            if (std::fabs(trueOffY - 1.0) < 0.25) {
                lineY_h.push_back(0.5 * (outY[static_cast<size_t>(144 * w + x)] + outY[static_cast<size_t>(145 * w + x)]));
            } else {
                lineY_h.push_back(outY[static_cast<size_t>(144 * w + x)]);
            }
        }
        for (int gx = 48; gx < 80; ++gx) {
            lineC_h.push_back(outC[static_cast<size_t>(72 * cw + gx)]);
        }
        const double crossY_x = 96.0 + findCross(lineY_h) + 0.5;
        const double crossC_x = 2.0 * (48.0 + findCross(lineC_h)) + trueOffX;

        std::vector<double> lineY_v, lineC_v;
        for (int y = 96; y < 160; ++y) {
            if (std::fabs(trueOffX - 1.0) < 0.25) {
                lineY_v.push_back(0.5 * (outY[static_cast<size_t>(y * w + 144)] + outY[static_cast<size_t>(y * w + 145)]));
            } else {
                lineY_v.push_back(outY[static_cast<size_t>(y * w + 144)]);
            }
        }
        for (int gy = 48; gy < 80; ++gy) {
            lineC_v.push_back(outC[static_cast<size_t>(gy * cw + 72)]);
        }
        const double crossY_y = 96.0 + findCross(lineY_v) + 0.5;
        const double crossC_y = 2.0 * (48.0 + findCross(lineC_v)) + trueOffY;

        return {std::fabs(crossY_x - crossC_x), std::fabs(crossY_y - crossC_y)};
    };

    // Legacy hardcoded Center(1.0, 1.0) on Left-sited (0.5, 1.0) 4:2:0 at 2.0x zoom leaks ~0.50 luma px (=0.25 source luma px)!
    const auto legacyErr = runCase(0.5, 1.0, 1.0, 1.0, 2.0, 0.0);
    assert(legacyErr.first > 0.25);

    // Fixed siting-aware sampling on Left-sited (0.5, 1.0), Center-sited (1.0, 1.0), and rotated 1.32x
    const auto leftIdErr = runCase(0.5, 1.0, 0.5, 1.0, 1.0, 0.0);
    const auto leftZoomErr = runCase(0.5, 1.0, 0.5, 1.0, 2.0, 0.0);
    const auto leftRotErr = runCase(0.5, 1.0, 0.5, 1.0, 1.32, 8.0);
    const auto centerZoomErr = runCase(1.0, 1.0, 1.0, 1.0, 2.0, 0.0);

    std::printf(
        "Chroma 4:2:0 Siting Errors -> LegacyLeft@2x=(%.4f, %.4f) | Left@1x=(%.4f, %.4f) | Left@2x=(%.4f, %.4f) | LeftRot8@1.32x=(%.4f, %.4f) | Center@2x=(%.4f, %.4f)\n",
        legacyErr.first, legacyErr.second,
        leftIdErr.first, leftIdErr.second,
        leftZoomErr.first, leftZoomErr.second,
        leftRotErr.first, leftRotErr.second,
        centerZoomErr.first, centerZoomErr.second
    );

    assert(leftIdErr.first < 0.05 && leftIdErr.second < 0.05);
    assert(leftZoomErr.first < 0.08 && leftZoomErr.second < 0.05);
    assert(leftRotErr.first < 0.10 && leftRotErr.second < 0.08);
    assert(centerZoomErr.first < 0.05 && centerZoomErr.second < 0.05);
}

void testPtsIntegrityValidation() {
    // 1. Uniform monotonic 30fps and 120fps streams
    std::vector<double> in30, out30;
    for (int i = 0; i < 180; ++i) {
        const double t = static_cast<double>(i) / 30.0;
        in30.push_back(t);
        out30.push_back(t);
    }
    const auto p30 = gf::QualityMetrics::validatePtsIntegrity(in30, out30, 30.0);
    assert(p30.passed);
    assert(p30.nonMonotonicCount == 0);
    assert(p30.duplicateFrameCount == 0);
    assert(p30.writerDroppedCount == 0);
    assert(p30.stdDevDtMs < 1e-4);
    assert(p30.maxInputOutputStepDriftMs < 1e-4);

    std::vector<double> in120, out120;
    for (int i = 0; i < 240; ++i) {
        const double t = static_cast<double>(i) / 120.0;
        in120.push_back(t);
        out120.push_back(t);
    }
    const auto p120 = gf::QualityMetrics::validatePtsIntegrity(in120, out120, 120.0);
    assert(p120.passed);
    assert(std::fabs(p120.meanDtMs - (1000.0 / 120.0)) < 1e-3);

    // 2. Duplicate / zero-dt frame detection
    auto outDup = out30;
    outDup[50] = outDup[49];
    const auto pDup = gf::QualityMetrics::validatePtsIntegrity(in30, outDup, 30.0);
    assert(!pDup.passed);
    assert(pDup.nonMonotonicCount >= 1);

    // 3. Near-duplicate frame (dt < 0.35 / fps)
    auto outNearDup = out30;
    outNearDup[50] = outNearDup[49] + 0.002;
    const auto pNearDup = gf::QualityMetrics::validatePtsIntegrity(in30, outNearDup, 30.0);
    assert(!pNearDup.passed);
    assert(pNearDup.duplicateFrameCount >= 1);

    // 4. Dropped frame by writer
    auto outDrop = out30;
    outDrop.erase(outDrop.begin() + 60);
    const auto pDrop = gf::QualityMetrics::validatePtsIntegrity(in30, outDrop, 30.0);
    assert(!pDrop.passed);
    assert(pDrop.writerDroppedCount == 1);

    // 5. Interval drift (jittery writer timestamps)
    auto outDrift = out30;
    outDrift[80] += 0.004;
    const auto pDrift = gf::QualityMetrics::validatePtsIntegrity(in30, outDrift, 30.0);
    assert(!pDrift.passed);
    assert(pDrift.maxInputOutputStepDriftMs > 3.5);

    std::printf(
        "PTS Integrity Test PASSED: 30fps(dt=%.3fms, drift=%.4fms), 120fps(dt=%.3fms) | Detected dup/drop/drift failures.\n",
        p30.meanDtMs, p30.maxInputOutputStepDriftMs, p120.meanDtMs
    );
}

void testTemporalSharpeningStability() {
    const int w = 64, h = 64;
    const int nFrames = 60;
    const double fps = 30.0;
    const double sharpenStrength = 0.75;

    std::vector<double> zoomCurve(static_cast<size_t>(nFrames), 1.0);
    for (int t = 0; t < nFrames; ++t) {
        zoomCurve[static_cast<size_t>(t)] = 1.28 + 0.14 * std::sin(2.0 * kPi * 4.0 * static_cast<double>(t) / fps);
    }
    const std::vector<double> zoomLPF = gf::QualityMetrics::lowPassFilterZoomSub1Hz(zoomCurve, fps, 0.65);
    assert(zoomLPF.size() == zoomCurve.size());

    double rawZoomMin = 10.0, rawZoomMax = 0.0;
    double lpfZoomMin = 10.0, lpfZoomMax = 0.0;
    for (int t = 6; t < nFrames - 6; ++t) {
        rawZoomMin = std::min(rawZoomMin, zoomCurve[static_cast<size_t>(t)]);
        rawZoomMax = std::max(rawZoomMax, zoomCurve[static_cast<size_t>(t)]);
        lpfZoomMin = std::min(lpfZoomMin, zoomLPF[static_cast<size_t>(t)]);
        lpfZoomMax = std::max(lpfZoomMax, zoomLPF[static_cast<size_t>(t)]);
    }
    assert((lpfZoomMax - lpfZoomMin) < 0.15 * (rawZoomMax - rawZoomMin));

    // Base synthetic image: flat mid-gray region [0..39] + crisp step edge (0.25 -> 0.75 at x=48)
    std::vector<double> baseImg(static_cast<size_t>(w * h), 0.42);
    for (int y = 0; y < h; ++y) {
        for (int x = 40; x < w; ++x) {
            baseImg[static_cast<size_t>(y * w + x)] = (x < 48) ? 0.25 : 0.75;
        }
    }

    auto xorshift = [](uint32_t& s) -> uint32_t {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        return s;
    };
    auto randNormal = [&](uint32_t& s) -> double {
        double sum = 0.0;
        for (int k = 0; k < 4; ++k) {
            sum += static_cast<double>(xorshift(s) & 0xFFFFu) / 65535.0;
        }
        return (sum - 2.0) * std::sqrt(3.0);
    };

    const int roiX0 = 12, roiX1 = 30, roiY0 = 16, roiY1 = 48;
    const size_t roiCount = static_cast<size_t>((roiX1 - roiX0) * (roiY1 - roiY0));
    std::vector<double> prevDeltaLeg(roiCount, 0.0);
    std::vector<double> prevDeltaNew(roiCount, 0.0);
    double flickerVarLeg = 0.0;
    double flickerVarNew = 0.0;
    size_t diffSamples = 0;

    for (int t = 0; t < nFrames; ++t) {
        std::vector<double> noisy = baseImg;
        uint32_t rng = 987654321u + static_cast<uint32_t>(t) * 2654435761u;
        for (size_t idx = 0; idx < noisy.size(); ++idx) {
            noisy[idx] = std::min(1.0, std::max(0.0, baseImg[idx] + 0.020 * randNormal(rng)));
        }

        const double jx = 0.20 * std::sin(2.1 * static_cast<double>(t));
        const double jy = 0.18 * std::cos(1.7 * static_cast<double>(t));

        size_t p = 0;
        for (int y = roiY0; y < roiY1; ++y) {
            for (int x = roiX0; x < roiX1; ++x, ++p) {
                const double sx = static_cast<double>(x) + jx;
                const double sy = static_cast<double>(y) + jy;
                const double valBase = sampleLanczos4CasCPU(noisy, w, h, sx, sy, 0.0, 1.0, false);
                const double valLeg = sampleLanczos4CasCPU(noisy, w, h, sx, sy, sharpenStrength, zoomCurve[static_cast<size_t>(t)], true);
                const double valNew = sampleLanczos4CasCPU(noisy, w, h, sx, sy, sharpenStrength, zoomLPF[static_cast<size_t>(t)], false);

                const double dLeg = valLeg - valBase;
                const double dNew = valNew - valBase;
                if (t > 0) {
                    const double diffLeg = dLeg - prevDeltaLeg[p];
                    const double diffNew = dNew - prevDeltaNew[p];
                    flickerVarLeg += diffLeg * diffLeg;
                    flickerVarNew += diffNew * diffNew;
                    ++diffSamples;
                }
                prevDeltaLeg[p] = dLeg;
                prevDeltaNew[p] = dNew;
            }
        }
    }

    flickerVarLeg /= static_cast<double>(std::max<size_t>(1, diffSamples));
    flickerVarNew /= static_cast<double>(std::max<size_t>(1, diffSamples));
    const double reductionPct = (1.0 - flickerVarNew / std::max(1e-18, flickerVarLeg)) * 100.0;

    // Verify step-edge contrast preservation at x=46..49 on noiseless frame
    const double baseEdgeLow = sampleLanczos4CasCPU(baseImg, w, h, 46.2, 32.0, 0.0, 1.0, false);
    const double baseEdgeHigh = sampleLanczos4CasCPU(baseImg, w, h, 48.8, 32.0, 0.0, 1.0, false);
    const double c2EdgeLow = sampleLanczos4CasCPU(baseImg, w, h, 46.2, 32.0, sharpenStrength, zoomLPF[30], false);
    const double c2EdgeHigh = sampleLanczos4CasCPU(baseImg, w, h, 48.8, 32.0, sharpenStrength, zoomLPF[30], false);
    const double edgeBase = baseEdgeHigh - baseEdgeLow;
    const double edgeC2 = c2EdgeHigh - c2EdgeLow;

    std::printf(
        "Temporal Sharpening Stability PASSED: FlickerVar %.2e -> %.2e (%.1f%% reduction) | EdgeContrast %.3f -> %.3f | LPF ZoomSpan %.3f -> %.3f\n",
        flickerVarLeg, flickerVarNew, reductionPct, edgeBase, edgeC2,
        rawZoomMax - rawZoomMin, lpfZoomMax - lpfZoomMin
    );
    assert(reductionPct > 50.0);
    assert(edgeC2 >= 0.95 * edgeBase);
}

} // namespace

int main() {
    double rotPsnr = 0.0, rotSsim = 0.0;
    testSinglePassResamplingFidelity(rotPsnr, rotSsim);
    testTemporalFullFrameEdgeFill();
    testFinalBenchmarkScorecard(rotPsnr, rotSsim);
    testQualityMetricsAndVideoAnalyzer();
    testChroma420SitingCheckerboard();
    testPtsIntegrityValidation();
    testTemporalSharpeningStability();
    return 0;
}
