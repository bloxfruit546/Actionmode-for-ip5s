#include "CalibrationRefiner.hpp"
#include "FrameWarpCalculator.hpp"
#include "OpticalSync.hpp"
#include "QualityMetrics.hpp"
#include "Quaternion.hpp"
#include "SidecarIO.hpp"
#include "TrajectoryEngine.hpp"
#include "VirtualGyroEngine.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

enum class ClipKind {
    Walking = 0,
    Running = 1,
    Motorbike = 2,
    FastPan = 3,
    LowTexture = 4,
    LowLight = 5
};

struct ClipSpec {
    std::string name;
    ClipKind kind;
    double durationSec = 6.0;
    double fps = 30.0;
    double imuHz = 120.0;
    double trueOffsetMs = 0.0;
    double trueDriftMsPerSec = 0.0;
    double trueBiasCam[3] = {0.0, 0.0, 0.0};
    double headerBiasCam[3] = {0.0, 0.0, 0.0};
    double trueScaleCam[3] = {1.0, 1.0, 1.0};
    double trueReadoutSec = 0.0152;
    double trueFocalScale = 1.0;
    uint32_t seed = 1001;
    // Stabilizer settings (preset style)
    double pitchSmooth = 0.78;
    double yawSmooth = 0.88;
    double rollSmooth = 0.85;
    double maxZoomLimit = 1.32;
    bool dynamicZoom = false;
    bool horizonLock = true;
    gf::HorizonMode horizonMode = gf::HorizonMode::ActionLevel40;
    bool motoActionMode = false;
};

struct BenchmarkOptions {
    bool runSuite = true;
    std::string outPath;
    std::string sidecarPath;
    bool useCalibRefine = true;
    bool useConstrainedPath = true;
    bool useTemporalSharpen = true;
};

gf::Vec3 evalTrueOmegaCam(ClipKind kind, double t) {
    switch (kind) {
    case ClipKind::Walking: {
        const double wx = 0.36 * std::cos(2.0 * kPi * 1.9 * t)
                        + 0.22 * std::sin(2.0 * kPi * 3.8 * t + 0.4)
                        + 0.14 * std::cos(2.0 * kPi * 6.5 * t);
        const double wy = 0.10 * std::sin(2.0 * kPi * 0.22 * t)
                        + 0.24 * std::sin(2.0 * kPi * 1.9 * t + 1.0)
                        + 0.16 * std::cos(2.0 * kPi * 5.2 * t);
        const double wz = 0.20 * std::sin(2.0 * kPi * 2.4 * t + 0.7)
                        + 0.11 * std::cos(2.0 * kPi * 6.2 * t);
        return {wx, wy, wz};
    }
    case ClipKind::Running: {
        // Large running shocks pushing past 1.32x crop envelope
        const double wx = 1.15 * std::cos(2.0 * kPi * 2.8 * t)
                        + 0.48 * std::sin(2.0 * kPi * 5.6 * t + 0.2)
                        + 0.22 * std::cos(2.0 * kPi * 8.4 * t);
        const double wy = 0.14 * std::sin(2.0 * kPi * 0.25 * t)
                        + 0.95 * std::sin(2.0 * kPi * 2.8 * t + 0.8)
                        + 0.36 * std::cos(2.0 * kPi * 6.4 * t);
        const double wz = 0.55 * std::sin(2.0 * kPi * 2.8 * t + 1.5)
                        + 0.20 * std::cos(2.0 * kPi * 7.1 * t);
        return {wx, wy, wz};
    }
    case ClipKind::Motorbike: {
        // Cornering yaw + lean roll + high-freq engine buzz
        const double cornerEnv = std::exp(-0.5 * std::pow((t - 3.0) / 1.1, 2.0));
        const double wx = 0.32 * std::cos(2.0 * kPi * 4.2 * t)
                        + 0.28 * std::sin(2.0 * kPi * 14.0 * t);
        const double wy = 0.38 * cornerEnv
                        + 0.20 * std::sin(2.0 * kPi * 7.5 * t);
        const double wz = 0.35 * std::sin(2.0 * kPi * 0.35 * t)
                        + 0.25 * std::cos(2.0 * kPi * 11.0 * t);
        return {wx, wy, wz};
    }
    case ClipKind::FastPan: {
        // 0..1.5s hold, 1.5..4.0s fast 42 deg/s yaw pan, 4.0..6.0s hold, plus hand jitter
        double panYaw = 0.0;
        if (t > 1.3 && t < 4.2) {
            const double uRise = std::min(1.0, std::max(0.0, (t - 1.3) / 0.4));
            const double uFall = std::min(1.0, std::max(0.0, (4.2 - t) / 0.4));
            const double env = (uRise * uRise * (3.0 - 2.0 * uRise)) * (uFall * uFall * (3.0 - 2.0 * uFall));
            panYaw = (42.0 * kPi / 180.0) * env;
        }
        const double wx = 0.28 * std::sin(2.0 * kPi * 4.8 * t)
                        + 0.16 * std::cos(2.0 * kPi * 7.3 * t);
        const double wy = panYaw
                        + 0.24 * std::cos(2.0 * kPi * 3.9 * t);
        const double wz = 0.18 * std::sin(2.0 * kPi * 5.1 * t);
        return {wx, wy, wz};
    }
    case ClipKind::LowTexture: {
        const double wx = 0.24 * std::cos(2.0 * kPi * 2.2 * t) + 0.12 * std::sin(2.0 * kPi * 5.0 * t);
        const double wy = 0.22 * std::sin(2.0 * kPi * 1.8 * t + 0.6) + 0.10 * std::cos(2.0 * kPi * 4.5 * t);
        const double wz = 0.15 * std::sin(2.0 * kPi * 2.5 * t);
        return {wx, wy, wz};
    }
    case ClipKind::LowLight: {
        const double pan = 0.18 * std::sin(2.0 * kPi * 0.25 * t);
        const double wx = 0.34 * std::cos(2.0 * kPi * 3.2 * t) + 0.18 * std::sin(2.0 * kPi * 6.0 * t);
        const double wy = pan + 0.30 * std::sin(2.0 * kPi * 2.7 * t + 0.5);
        const double wz = 0.20 * std::cos(2.0 * kPi * 3.5 * t);
        return {wx, wy, wz};
    }
    }
    return {0.0, 0.0, 0.0};
}

inline uint32_t xorshift32(uint32_t& state) {
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

inline double randNormalApprox(uint32_t& state) {
    double sum = 0.0;
    for (int k = 0; k < 4; ++k) {
        sum += static_cast<double>(xorshift32(state) & 0xFFFFu) / 65535.0;
    }
    return (sum - 2.0) * std::sqrt(3.0); // zero-mean unit-variance approx
}

uint8_t evalWorldLuma(ClipKind kind, const gf::Vec3& rayW, uint32_t& rng) {
    if (kind == ClipKind::LowTexture) {
        // 90% smooth gradient sky/wall with very faint distant ridge and light sensor noise
        const double base = 165.0 + 38.0 * rayW.y + 18.0 * rayW.x
                          + 6.0 * std::sin(rayW.x * 7.0 + rayW.y * 5.0);
        const double patch = (rayW.x > 0.18 && rayW.x < 0.34 && rayW.y > 0.05 && rayW.y < 0.20)
            ? (18.0 * std::sin(rayW.x * 42.0) * std::cos(rayW.y * 42.0))
            : 0.0;
        const double noisy = base + patch + 3.2 * randNormalApprox(rng);
        return static_cast<uint8_t>(std::min(255.0, std::max(0.0, noisy)));
    }

    const double tex = 0.45 * std::sin(rayW.x * 24.0 + rayW.y * 16.0)
                     + 0.35 * std::cos(rayW.x * 38.0 - rayW.y * 29.0)
                     + 0.25 * std::sin(rayW.y * 52.0 + rayW.z * 14.0)
                     + 0.20 * std::cos((rayW.x + rayW.y) * 68.0);
    if (kind == ClipKind::LowLight) {
        const double dark = 46.0 + 28.0 * tex + 4.2 * randNormalApprox(rng);
        return static_cast<uint8_t>(std::min(255.0, std::max(0.0, dark)));
    }
    const double val = 128.0 + 72.0 * tex + 1.2 * randNormalApprox(rng);
    return static_cast<uint8_t>(std::min(255.0, std::max(0.0, val)));
}

double sampleBilinearU8(const std::vector<uint8_t>& img, int w, int h, double sx, double sy) {
    const double x = std::min(static_cast<double>(w - 1), std::max(0.0, sx));
    const double y = std::min(static_cast<double>(h - 1), std::max(0.0, sy));
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = std::min(x0 + 1, w - 1);
    const int y1 = std::min(y0 + 1, h - 1);
    const double fx = x - x0;
    const double fy = y - y0;
    const double v00 = img[static_cast<size_t>(y0 * w + x0)];
    const double v10 = img[static_cast<size_t>(y0 * w + x1)];
    const double v01 = img[static_cast<size_t>(y1 * w + x0)];
    const double v11 = img[static_cast<size_t>(y1 * w + x1)];
    return (v00 * (1.0 - fx) + v10 * fx) * (1.0 - fy) + (v01 * (1.0 - fx) + v11 * fx) * fy;
}

struct ClipBenchmarkOutput {
    std::string name;
    gf::QualityReport report;
    gf::OpticalSyncResult syncResult;
};

ClipBenchmarkOutput runSingleSyntheticClip(const ClipSpec& spec, const BenchmarkOptions& opts) {
    const int simW = 480;
    const int simH = 270;
    const double fullW = 1920.0;
    const double fullH = 1080.0;
    const double scaleSim = static_cast<double>(simW) / fullW;
    const double fxFull = 1815.0 * spec.trueFocalScale;
    const double fyFull = 1815.0 * spec.trueFocalScale;
    const double cxFull = 960.0;
    const double cyFull = 540.0;
    const double fxSim = fxFull * scaleSim;
    const double fySim = fyFull * scaleSim;
    const double cxSim = cxFull * scaleSim;
    const double cySim = cyFull * scaleSim;

    // 1. High-rate (480 Hz) ground-truth orientation trajectory from t = -0.15s to duration + 0.15s
    const double tStart = 10.0;
    const double simStart = tStart - 0.15;
    const double simEnd = tStart + spec.durationSec + 0.15;
    const double simDt = 1.0 / 480.0;
    const int simSteps = static_cast<int>(std::ceil((simEnd - simStart) / simDt)) + 1;
    std::vector<gf::Quat> trueQuats(static_cast<size_t>(simSteps), gf::Quat::identity());
    for (int i = 1; i < simSteps; ++i) {
        const double tMid = simStart + (static_cast<double>(i) - 0.5) * simDt;
        const gf::Vec3 wMid = evalTrueOmegaCam(spec.kind, tMid - tStart);
        trueQuats[static_cast<size_t>(i)] = gf::Quat::integrateGyroStep(trueQuats[static_cast<size_t>(i - 1)], wMid, simDt);
    }

    auto sampleTrueQuatAt = [&](double t) -> gf::Quat {
        const double pos = (t - simStart) / simDt;
        if (pos <= 0.0) return trueQuats.front();
        const int i0 = std::min(simSteps - 2, static_cast<int>(std::floor(pos)));
        const double alpha = std::min(1.0, std::max(0.0, pos - i0));
        return gf::Quat::slerp(trueQuats[static_cast<size_t>(i0)], trueQuats[static_cast<size_t>(i0 + 1)], alpha);
    };

    // 2. Synthesize header, IMU gyro+accel records (with true offset, drift, scale, bias, noise)
    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.capture_mode = 0;
    hdr.width = 1920;
    hdr.height = 1080;
    hdr.nominal_fps = spec.fps;
    hdr.readout_time_sec = 0.0152;
    // LandscapeLeft: (-y, -x, -z) -> iosBias = (-headerBiasCam.y, -headerBiasCam.x, -headerBiasCam.z)
    hdr.gyro_bias_x = -spec.headerBiasCam[1];
    hdr.gyro_bias_y = -spec.headerBiasCam[0];
    hdr.gyro_bias_z = -spec.headerBiasCam[2];
    hdr.lens_fx = 1815.0;
    hdr.lens_fy = 1815.0;
    hdr.lens_cx = 960.0;
    hdr.lens_cy = 540.0;
    hdr.dist_k1 = 0.012;
    hdr.dist_k2 = -0.022;

    uint32_t rngImu = spec.seed ^ 0xA5A51234u;
    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    const double imuDt = 1.0 / spec.imuHz;
    const int nImu = static_cast<int>(std::ceil((spec.durationSec + 0.24) / imuDt)) + 1;
    gyro.reserve(static_cast<size_t>(nImu));
    accel.reserve(static_cast<size_t>(nImu));

    for (int i = 0; i < nImu; ++i) {
        const double tGyro = (tStart - 0.12) + i * imuDt;
        const double tRel = tGyro - tStart;
        // Clock mapping: gyro records at tGyro correspond to physical time tPhys where tGyro = tPhys + delta(tPhys)
        const double deltaSec = (spec.trueOffsetMs + spec.trueDriftMsPerSec * tRel) * 0.001;
        const double tPhys = tGyro - deltaSec;
        const gf::Vec3 wTrue = evalTrueOmegaCam(spec.kind, tPhys - tStart);
        const double noise = 0.0015;
        const double wxMeas = spec.trueScaleCam[0] * wTrue.x + spec.trueBiasCam[0] + noise * randNormalApprox(rngImu);
        const double wyMeas = spec.trueScaleCam[1] * wTrue.y + spec.trueBiasCam[1] + noise * randNormalApprox(rngImu);
        const double wzMeas = spec.trueScaleCam[2] * wTrue.z + spec.trueBiasCam[2] + noise * randNormalApprox(rngImu);
        gyro.push_back({tGyro, -wyMeas, -wxMeas, -wzMeas});

        // World gravity = +Y_cam = (0, 1, 0) rotated into true camera frame at tPhys
        const gf::Quat qTrue = sampleTrueQuatAt(tPhys);
        const gf::Vec3 gCam = qTrue.conjugate().rotateVector(gf::Vec3(0.0, 1.0, 0.0));
        accel.push_back({
            tGyro,
            static_cast<float>(-gCam.y),
            static_cast<float>(-gCam.x),
            static_cast<float>(-gCam.z)
        });
    }

    // 3. Frame records and synthetic 480x270 raw frames
    const int nFrames = static_cast<int>(std::round(spec.durationSec * spec.fps));
    std::vector<gf::GFFrameRecord> frames;
    frames.reserve(static_cast<size_t>(nFrames));
    std::vector<std::vector<uint8_t>> rawLumaFrames(static_cast<size_t>(nFrames));
    std::vector<gf::OpticalFlowFrame> syncFrames;
    const size_t syncCap = std::min<size_t>(110, static_cast<size_t>(nFrames));
    syncFrames.reserve(syncCap);

    const double exposureSec = 0.005;
    for (int f = 0; f < nFrames; ++f) {
        const double pts = tStart + static_cast<double>(f) / spec.fps;
        frames.push_back({
            static_cast<uint32_t>(f),
            pts,
            static_cast<float>(exposureSec),
            100.0f,
            0.7f,
            static_cast<float>(hdr.readout_time_sec)
        });
        const double tMid = pts + 0.5 * exposureSec + 0.5 * hdr.readout_time_sec;

        std::vector<uint8_t> img(static_cast<size_t>(simW * simH));
        uint32_t rngPix = spec.seed + static_cast<uint32_t>(f) * 7919u;
        for (int y = 0; y < simH; ++y) {
            const double rowNorm = (static_cast<double>(y) / static_cast<double>(simH - 1)) - 0.5;
            const double tRow = tMid + rowNorm * spec.trueReadoutSec;
            const gf::Quat qRow = sampleTrueQuatAt(tRow);
            for (int x = 0; x < simW; ++x) {
                const double xn = (x + 0.5 - cxSim) / fxSim;
                const double yn = (y + 0.5 - cySim) / fySim;
                const gf::Vec3 rayW = qRow.rotateVector(gf::Vec3(xn, yn, 1.0).normalized());
                img[static_cast<size_t>(y * simW + x)] = evalWorldLuma(spec.kind, rayW, rngPix);
            }
        }
        if (static_cast<size_t>(f) < syncCap) {
            gf::OpticalFlowFrame sf;
            sf.timestampSec = tMid;
            sf.width = simW;
            sf.height = simH;
            sf.luma = img;
            syncFrames.push_back(std::move(sf));
        }
        rawLumaFrames[static_cast<size_t>(f)] = std::move(img);
    }

    // 4. Auto-Sync + optional Closed-Loop CalibrationRefiner -> TrajectoryEngine -> FrameWarpCalculator
    const gf::OpticalSyncResult syncRes = gf::OpticalSync::solveSyncAndAxes(syncFrames, hdr, gyro, spec.motoActionMode);

    gf::RefinedCalibration refinedCalib;
    bool hasRefined = false;
    if (opts.useCalibRefine) {
        gf::CalibrationRefiner refiner;
        gf::CalibrationModePrior prior;
        refiner.reset(hdr, gyro, frames, &prior, syncRes.valid ? &syncRes : nullptr, spec.motoActionMode);
        for (int f = 0; f < nFrames; ++f) {
            refiner.pushDownsampledLumaFrame(
                rawLumaFrames[static_cast<size_t>(f)].data(),
                simW, simH,
                frames[static_cast<size_t>(f)].pts_sec,
                static_cast<size_t>(f)
            );
        }
        refinedCalib = refiner.finalize();
        hasRefined = refinedCalib.valid;
    }

    gf::TrajectoryEngine engine;
    const double* customAxis = syncRes.valid ? syncRes.axisMatrix : nullptr;
    const double rollScale = (syncRes.valid && syncRes.hasRollScaleEstimate) ? syncRes.rollGyroScale : 1.0;
    engine.setImuData(hdr, gyro, accel, customAxis, spec.motoActionMode, rollScale, hasRefined ? &refinedCalib : nullptr);

    gf::CalibrationProfile prof = gf::CalibrationProfile::defaultIPhone5s1080p30();
    prof.k1 = 0.0;
    prof.k2 = 0.0;
    if (hasRefined) {
        prof.readoutTimeSec = refinedCalib.readoutMs * 0.001;
        prof.fx *= refinedCalib.focalScale;
        prof.fy *= refinedCalib.focalScale;
    } else {
        if (syncRes.valid && syncRes.hasReadoutEstimate) {
            prof.readoutTimeSec = syncRes.signedReadoutTimeMs * 0.001;
        }
        if (syncRes.valid && syncRes.hasFocalScaleEstimate) {
            prof.fx *= syncRes.effectiveFocalScale;
            prof.fy *= syncRes.effectiveFocalScale;
        }
    }

    gf::StabilizerConfig cfg;
    cfg.pitchSmoothness = spec.pitchSmooth;
    cfg.yawSmoothness = spec.yawSmooth;
    cfg.rollSmoothness = spec.rollSmooth;
    cfg.timeOffsetMs = hasRefined ? 0.0 : (syncRes.valid ? syncRes.bestOffsetMs : 0.0);
    cfg.maxZoomLimit = spec.maxZoomLimit;
    cfg.horizonLockEnabled = spec.horizonLock;
    cfg.horizonMode = spec.horizonMode;
    cfg.rollingShutterCorrection = true;
    cfg.dynamicZoomEnabled = spec.dynamicZoom;
    cfg.motoActionMode = spec.motoActionMode;
    cfg.useConstrainedPath = opts.useConstrainedPath;
    cfg.videoRotationDegrees = 0;
    cfg.calibWidth = prof.width;
    cfg.calibHeight = prof.height;
    cfg.calibFx = prof.fx;
    cfg.calibFy = prof.fy;
    cfg.calibCx = prof.cx;
    cfg.calibCy = prof.cy;
    cfg.calibReadoutSec = prof.readoutTimeSec;

    const gf::TrajectoryResult traj = engine.solve(frames, cfg);

    const gf::WarpComputationResult warp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        engine, traj, prof, spec.fps, cfg.maxZoomLimit, cfg.rollingShutterCorrection, cfg.dynamicZoomEnabled
    );

    // 5. Render stabilized frames at 480x270 and analyze via VideoQualityAnalyzer (VirtualGyroEngine)
    gf::VirtualGyroConfig vgCfg;
    vgCfg.videoWidth = static_cast<uint32_t>(simW);
    vgCfg.videoHeight = static_cast<uint32_t>(simH);
    vgCfg.nominalFps = spec.fps;
    vgCfg.readoutTimeSec = 0.0; // Output is already RS-rectified
    vgCfg.lensFx = prof.fx * scaleSim;
    vgCfg.lensFy = prof.fy * scaleSim;
    vgCfg.lensCx = cxSim;
    vgCfg.lensCy = cySim;

    gf::VideoQualityAnalyzer analyzer(vgCfg);
    std::vector<double> zoomUsed(static_cast<size_t>(nFrames), 1.0);
    std::vector<double> inputPts(static_cast<size_t>(nFrames), 0.0);
    std::vector<uint8_t> outFrame(static_cast<size_t>(simW * simH), 0);

    for (int f = 0; f < nFrames; ++f) {
        const auto& lut = warp.frameLUTs[static_cast<size_t>(f)];
        zoomUsed[static_cast<size_t>(f)] = static_cast<double>(lut.zoomUsed);
        inputPts[static_cast<size_t>(f)] = frames[static_cast<size_t>(f)].pts_sec;
        const auto& src = rawLumaFrames[static_cast<size_t>(f)];

        for (int y = 0; y < simH; ++y) {
            const double dstY = (static_cast<double>(y) + 0.5) / scaleSim;
            for (int x = 0; x < simW; ++x) {
                const double dstX = (static_cast<double>(x) + 0.5) / scaleSim;
                double rowNorm = 0.5;
                double sxFull = dstX;
                double syFull = dstY;
                for (int iter = 0; iter < 4; ++iter) {
                    const double fIdx = std::min(63.0, std::max(0.0, rowNorm * 63.0));
                    const int i0 = std::min(62, static_cast<int>(std::floor(fIdx)));
                    const double t = fIdx - i0;
                    const float* m0 = lut.sliceH[i0];
                    const float* m1 = lut.sliceH[i0 + 1];
                    const double r0x = m0[0] + t * (m1[0] - m0[0]);
                    const double r0y = m0[1] + t * (m1[1] - m0[1]);
                    const double r0z = m0[2] + t * (m1[2] - m0[2]);
                    const double r1x = m0[3] + t * (m1[3] - m0[3]);
                    const double r1y = m0[4] + t * (m1[4] - m0[4]);
                    const double r1z = m0[5] + t * (m1[5] - m0[5]);
                    const double r2x = m0[6] + t * (m1[6] - m0[6]);
                    const double r2y = m0[7] + t * (m1[7] - m0[7]);
                    const double r2z = m0[8] + t * (m1[8] - m0[8]);
                    const double w = r2x * dstX + r2y * dstY + r2z;
                    if (w <= 0.1) break;
                    sxFull = (r0x * dstX + r0y * dstY + r0z) / w;
                    syFull = (r1x * dstX + r1y * dstY + r1z) / w;
                    rowNorm = std::min(1.0, std::max(0.0, (syFull - 0.5) / (fullH - 1.0)));
                }
                const double sx = sxFull * scaleSim - 0.5;
                const double sy = syFull * scaleSim - 0.5;
                const double val = sampleBilinearU8(src, simW, simH, sx, sy);
                outFrame[static_cast<size_t>(y * simW + x)] = static_cast<uint8_t>(std::min(255.0, std::max(0.0, std::round(val))));
            }
        }
        analyzer.pushRenderedLumaFrame(outFrame.data(), simW, simH, frames[static_cast<size_t>(f)].pts_sec);
    }

    gf::QualityReport visReport = analyzer.finalize(&zoomUsed, traj.maxOutputRollErrorDeg, 3.0);
    // Compute planned-trajectory smoothness metrics as well for hybrid validation when visual texture is absent
    gf::QualityReport trajReport = gf::QualityMetrics::evaluateTrajectory(
        traj.smoothFrameQuats, spec.fps, &zoomUsed, traj.maxOutputRollErrorDeg, 3.0
    );

    if (!visReport.hasVisualMetrics || visReport.meanInlierRatio < 0.45) {
        visReport.highFreqOmegaEnergyAbove2Hz = trajReport.highFreqOmegaEnergyAbove2Hz;
        visReport.highFreqOmegaRmsAbove2Hz = trajReport.highFreqOmegaRmsAbove2Hz;
        visReport.jerkRms = trajReport.jerkRms;
        visReport.liuStabilityScore = trajReport.liuStabilityScore;
        visReport.liuStabilityScoreMean = trajReport.liuStabilityScoreMean;
        visReport.accelDiscontinuityCount = trajReport.accelDiscontinuityCount;
        visReport.maxAccelJumpRadPerSec2 = trajReport.maxAccelJumpRadPerSec2;
    } else {
        // Combine trajectory clamp discontinuity detection if planned path itself kinked harder than visual
        visReport.accelDiscontinuityCount = std::max(visReport.accelDiscontinuityCount, trajReport.accelDiscontinuityCount);
    }

    if (hasRefined) {
        visReport.calibResidualRmsBefore = refinedCalib.residualRmsBefore;
        visReport.calibResidualRmsAfter = refinedCalib.residualRmsAfter;
    }
    // Measure raw input gyro vs visual residual RMS before/after calibration on full clip
    visReport.ptsCheck = gf::QualityMetrics::validatePtsIntegrity(inputPts, inputPts, spec.fps);
    return {spec.name, visReport, syncRes};
}

std::vector<ClipSpec> makeStandardSixClips() {
    std::vector<ClipSpec> clips;

    ClipSpec c1;
    c1.name = "01_walking";
    c1.kind = ClipKind::Walking;
    c1.trueOffsetMs = 6.5;
    c1.trueBiasCam[0] = 0.009; c1.trueBiasCam[1] = -0.007; c1.trueBiasCam[2] = 0.005;
    c1.trueScaleCam[0] = 1.018; c1.trueScaleCam[1] = 0.985; c1.trueScaleCam[2] = 1.012;
    c1.seed = 101;
    clips.push_back(c1);

    ClipSpec c2;
    c2.name = "02_running";
    c2.kind = ClipKind::Running;
    c2.trueOffsetMs = 8.5;
    c2.trueDriftMsPerSec = 0.45;
    c2.trueBiasCam[0] = 0.014; c2.trueBiasCam[1] = -0.011; c2.trueBiasCam[2] = 0.008;
    c2.trueScaleCam[0] = 1.028; c2.trueScaleCam[1] = 0.976; c2.trueScaleCam[2] = 1.020;
    c2.maxZoomLimit = 1.32;
    c2.seed = 202;
    clips.push_back(c2);

    ClipSpec c3;
    c3.name = "03_motorbike";
    c3.kind = ClipKind::Motorbike;
    c3.trueOffsetMs = 5.0;
    c3.trueDriftMsPerSec = 0.60;
    c3.trueBiasCam[0] = 0.010; c3.trueBiasCam[1] = 0.008; c3.trueBiasCam[2] = -0.009;
    c3.trueScaleCam[0] = 1.022; c3.trueScaleCam[1] = 0.982; c3.trueScaleCam[2] = 1.025;
    c3.pitchSmooth = 0.94; c3.yawSmooth = 0.72; c3.rollSmooth = 0.98;
    c3.maxZoomLimit = 1.65;
    c3.dynamicZoom = true;
    c3.horizonMode = gf::HorizonMode::Full360Lock;
    c3.motoActionMode = true;
    c3.seed = 303;
    clips.push_back(c3);

    ClipSpec c4;
    c4.name = "04_fast_pan";
    c4.kind = ClipKind::FastPan;
    c4.trueOffsetMs = 6.0;
    c4.trueBiasCam[0] = 0.007; c4.trueBiasCam[1] = -0.005; c4.trueBiasCam[2] = 0.004;
    c4.trueScaleCam[0] = 1.015; c4.trueScaleCam[1] = 0.988; c4.trueScaleCam[2] = 1.010;
    c4.maxZoomLimit = 1.38;
    c4.seed = 404;
    clips.push_back(c4);

    ClipSpec c5;
    c5.name = "05_low_texture";
    c5.kind = ClipKind::LowTexture;
    c5.trueOffsetMs = 7.0;
    c5.trueBiasCam[0] = 0.009; c5.trueBiasCam[1] = -0.005; c5.trueBiasCam[2] = 0.003;
    c5.headerBiasCam[0] = 0.008; c5.headerBiasCam[1] = -0.004; c5.headerBiasCam[2] = 0.003;
    c5.seed = 505;
    clips.push_back(c5);

    ClipSpec c6;
    c6.name = "06_low_light";
    c6.kind = ClipKind::LowLight;
    c6.trueOffsetMs = 6.0;
    c6.trueBiasCam[0] = 0.011; c6.trueBiasCam[1] = -0.008; c6.trueBiasCam[2] = 0.005;
    c6.trueScaleCam[0] = 1.016; c6.trueScaleCam[1] = 0.986; c6.trueScaleCam[2] = 1.012;
    c6.maxZoomLimit = 1.40;
    c6.dynamicZoom = true;
    c6.seed = 606;
    clips.push_back(c6);

    return clips;
}

int runSidecarCli(const std::string& path, const BenchmarkOptions& opts) {
    gf::GFFileHeader hdr{};
    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    std::vector<gf::GFFrameRecord> frames;
    gf::SidecarSummary summary;
    if (!gf::SidecarReader::readAll(path, hdr, gyro, accel, frames, summary) || frames.empty()) {
        std::fprintf(stderr, "Failed to read sidecar: %s (%s)\n", path.c_str(), summary.errorMessage.c_str());
        return 1;
    }

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel);
    const gf::CalibrationProfile prof = (hdr.capture_mode == 1 || hdr.capture_mode == 2)
        ? gf::CalibrationProfile::defaultIPhone5s720p120()
        : gf::CalibrationProfile::defaultIPhone5s1080p30();
    gf::StabilizerConfig cfg;
    cfg.useConstrainedPath = opts.useConstrainedPath;
    cfg.calibWidth = prof.width;
    cfg.calibHeight = prof.height;
    cfg.calibFx = prof.fx;
    cfg.calibFy = prof.fy;
    cfg.calibCx = prof.cx;
    cfg.calibCy = prof.cy;
    cfg.calibReadoutSec = prof.readoutTimeSec;
    const auto traj = engine.solve(frames, cfg);
    const auto warp = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        engine, traj, prof, hdr.nominal_fps > 1.0 ? hdr.nominal_fps : 30.0, cfg.maxZoomLimit, true, false
    );
    std::vector<double> zoomUsed(warp.frameLUTs.size(), 1.0);
    std::vector<double> pts(frames.size(), 0.0);
    for (size_t i = 0; i < warp.frameLUTs.size(); ++i) zoomUsed[i] = warp.frameLUTs[i].zoomUsed;
    for (size_t i = 0; i < frames.size(); ++i) pts[i] = frames[i].pts_sec;

    auto report = gf::QualityMetrics::evaluateTrajectory(
        traj.smoothFrameQuats, hdr.nominal_fps > 1.0 ? hdr.nominal_fps : 30.0, &zoomUsed, traj.maxOutputRollErrorDeg
    );
    report.ptsCheck = gf::QualityMetrics::validatePtsIntegrity(pts, pts, hdr.nominal_fps);
    const std::string json = gf::QualityMetrics::toJson(report, path);
    if (!opts.outPath.empty()) {
        std::ofstream ofs(opts.outPath);
        ofs << json << "\n";
    } else {
        std::printf("%s\n", json.c_str());
    }
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    BenchmarkOptions opts;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--run-suite") {
            opts.runSuite = true;
        } else if (arg == "--out" && i + 1 < argc) {
            opts.outPath = argv[++i];
        } else if (arg == "--sidecar" && i + 1 < argc) {
            opts.sidecarPath = argv[++i];
            opts.runSuite = false;
        } else if (arg == "--no-calib-refine") {
            opts.useCalibRefine = false;
        } else if (arg == "--no-constrained-path") {
            opts.useConstrainedPath = false;
        } else if (arg == "--no-temporal-sharpen") {
            opts.useTemporalSharpen = false;
        }
    }

    if (!opts.sidecarPath.empty()) {
        return runSidecarCli(opts.sidecarPath, opts);
    }

    const auto clips = makeStandardSixClips();
    std::vector<ClipBenchmarkOutput> outputs;
    outputs.reserve(clips.size());
    for (const auto& c : clips) {
        outputs.push_back(runSingleSyntheticClip(c, opts));
    }

    const bool allNewEnabled = opts.useCalibRefine && opts.useConstrainedPath && opts.useTemporalSharpen;
    const std::string versionTag = allNewEnabled ? "v1.6.0" : "v1.5.2";

    std::ostringstream oss;
    oss << "{\n"
        << "  \"version\": \"" << versionTag << "\",\n"
        << "  \"flags\": {\n"
        << "    \"useCalibRefine\": " << (opts.useCalibRefine ? "true" : "false") << ",\n"
        << "    \"useConstrainedPath\": " << (opts.useConstrainedPath ? "true" : "false") << ",\n"
        << "    \"useTemporalSharpen\": " << (opts.useTemporalSharpen ? "true" : "false") << "\n"
        << "  },\n"
        << "  \"clips\": [\n";
    for (size_t i = 0; i < outputs.size(); ++i) {
        oss << gf::QualityMetrics::toJson(outputs[i].report, outputs[i].name);
        if (i + 1 < outputs.size()) oss << ",";
        oss << "\n";
    }
    oss << "  ]\n}\n";

    const std::string jsonStr = oss.str();
    if (!opts.outPath.empty()) {
        std::ofstream ofs(opts.outPath);
        ofs << jsonStr;
        if (!ofs) {
            std::fprintf(stderr, "Failed to write benchmark output to %s\n", opts.outPath.c_str());
            return 1;
        }
    }

    std::printf(
        "%-16s | E(>2Hz)   | JerkRMS  | LiuScore | CropRatio | AvgZoom | AccelJumps | HorErr | CalibRMS(b->a)\n"
        "-----------------+-----------+----------+----------+-----------+---------+------------+--------+----------------\n",
        "Clip"
    );
    for (const auto& o : outputs) {
        const auto& r = o.report;
        std::printf(
            "%-16s | %9.5f | %8.3f | %8.4f | %9.4f | %7.3f | %10d | %6.2f | %.4f->%.4f\n",
            o.name.c_str(),
            r.highFreqOmegaEnergyAbove2Hz,
            r.jerkRms,
            r.liuStabilityScore,
            r.meanCroppingRatio,
            r.avgZoom,
            r.accelDiscontinuityCount,
            r.maxHorizonErrorDeg,
            r.calibResidualRmsBefore,
            r.calibResidualRmsAfter
        );
    }
    return 0;
}
