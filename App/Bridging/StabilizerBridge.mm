#import "StabilizerBridge.h"
#import "CalibrationProfile.hpp"
#import "FrameWarpCalculator.hpp"
#import "OpticalSync.hpp"
#import "SidecarIO.hpp"
#import "TrajectoryEngine.hpp"

#import <Metal/Metal.h>
#include <simd/simd.h>
#include <vector>

namespace {

struct GFMtlRowSlice {
    simd_float4 hRow0;
    simd_float4 hRow1;
    simd_float4 hRow2;
};

struct GFWarpFrameUniformsMetal {
    GFMtlRowSlice slices[64];
    GFMtlRowSlice histH[3];
    simd_float4 histGains;
    simd_float4 polishParams;
    simd_uint2 outSize;
    simd_uint2 inSize;
    simd_uint2 lumaOutSize;
    simd_uint2 lumaInSize;
    simd_float4 intrinsics;
    simd_float4 distortion;
};

struct GFHistSlot {
    CVPixelBufferRef pb = nullptr;
    NSUInteger frameIndex = 0;
};

} // namespace

@implementation GFOpticalSyncResultObjC
- (instancetype)init {
    self = [super init];
    if (self) {
        _axisPresetName = @"";
        _diagnosticMessage = @"";
    }
    return self;
}
@end

@implementation GFOpticalSyncCollectorBridge {
    std::vector<gf::OpticalFlowFrame> _frames;
}

- (void)reset {
    _frames.clear();
}

- (void)addLumaPixelBuffer:(CVPixelBufferRef)pixelBuffer timestampSec:(double)ts {
    if (!pixelBuffer) {
        return;
    }
    CVPixelBufferLockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
    const size_t srcW = CVPixelBufferGetWidthOfPlane(pixelBuffer, 0);
    const size_t srcH = CVPixelBufferGetHeightOfPlane(pixelBuffer, 0);
    const size_t stride = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0);
    const uint8_t *base = (const uint8_t *)CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0);

    if (base && srcW >= 64 && srcH >= 64) {
        const int ds = 4;
        const int dstW = (int)(srcW / ds);
        const int dstH = (int)(srcH / ds);
        gf::OpticalFlowFrame f;
        f.timestampSec = ts;
        f.width = dstW;
        f.height = dstH;
        f.luma.resize((size_t)(dstW * dstH));

        for (int y = 0; y < dstH; ++y) {
            const uint8_t *srcRow = base + (size_t)(y * ds) * stride;
            uint8_t *dstRow = f.luma.data() + (size_t)(y * dstW);
            for (int x = 0; x < dstW; ++x) {
                dstRow[x] = srcRow[x * ds];
            }
        }
        _frames.push_back(std::move(f));
    }
    CVPixelBufferUnlockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
}

- (GFOpticalSyncResultObjC *)solveWithSidecarPath:(NSString *)sidecarPath motoActionMode:(BOOL)motoActionMode {
    GFOpticalSyncResultObjC *out = [[GFOpticalSyncResultObjC alloc] init];

    gf::GFFileHeader hdr{};
    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    std::vector<gf::GFFrameRecord> scFrames;
    gf::SidecarSummary summary;

    if (!gf::SidecarReader::readAll([sidecarPath UTF8String], hdr, gyro, accel, scFrames, summary) ||
        !summary.validHeader) {
        out.valid = NO;
        out.diagnosticMessage = @"Invalid sidecar file.";
        return out;
    }

    // Align OpticalFlowFrame timestamps with exact exposure+readout center (unified with TrajectoryEngine)
    const size_t count = std::min(_frames.size(), scFrames.size());
    if (count > 0 && !_frames.empty()) {
        const double pts0 = scFrames[0].pts_sec
            + 0.5 * static_cast<double>(scFrames[0].exposure_duration_sec)
            + 0.5 * static_cast<double>(scFrames[0].rolling_shutter_sec);
        const double vid0 = _frames[0].timestampSec;
        const double delta = pts0 - vid0;
        for (size_t i = 0; i < _frames.size(); ++i) {
            if (i < scFrames.size()) {
                _frames[i].timestampSec = scFrames[i].pts_sec
                    + 0.5 * static_cast<double>(scFrames[i].exposure_duration_sec)
                    + 0.5 * static_cast<double>(scFrames[i].rolling_shutter_sec);
            } else {
                _frames[i].timestampSec += delta;
            }
        }
    }

    const gf::OpticalSyncResult res = gf::OpticalSync::solveSyncAndAxes(_frames, hdr, gyro, motoActionMode ? true : false);
    out.valid = res.valid ? YES : NO;
    out.bestOffsetMs = res.bestOffsetMs;
    out.bestNCC = res.bestNCC;
    out.bestAxisPresetIndex = res.bestAxisPresetIndex;
    out.axisPresetName = [NSString stringWithUTF8String:res.axisPresetName.c_str()] ?: @"";
    out.signedReadoutTimeMs = res.signedReadoutTimeMs;
    out.hasReadoutEstimate = res.hasReadoutEstimate ? YES : NO;
    out.effectiveFocalScale = res.effectiveFocalScale;
    out.hasFocalScaleEstimate = res.hasFocalScaleEstimate ? YES : NO;
    out.rollGyroScale = res.rollGyroScale;
    out.hasRollScaleEstimate = res.hasRollScaleEstimate ? YES : NO;
    out.diagnosticMessage = [NSString stringWithUTF8String:res.diagnosticMessage.c_str()] ?: @"";
    return out;
}

@end

@implementation GFStabilizerPlanObjC
- (instancetype)init {
    self = [super init];
    if (self) {
        _detectedOrientationName = @"Landscape (0°)";
        _gpuMatrixProof = @"";
        _errorMessage = @"";
    }
    return self;
}
@end

@implementation GFStabilizerBridge {
    id<MTLDevice> _device;
    id<MTLCommandQueue> _commandQueue;
    id<MTLComputePipelineState> _lumaPSO;
    id<MTLComputePipelineState> _chromaPSO;
    CVMetalTextureCacheRef _textureCache;
    std::vector<gf::GFWarpFrameLUT> _frameLUTs;
    std::vector<gf::Quat> _adjustedSmoothQuats;
    std::vector<gf::GFFrameRecord> _sidecarFrames;
    std::vector<double> _residualShiftX;
    std::vector<double> _residualShiftY;
    gf::CalibrationProfile _profile;
    gf::TrajectoryEngine _engine;
    gf::TrajectoryResult _traj;
    gf::StabilizerConfig _cfg;
    gf::HybridResidualTracker _hybridTracker;
    double _fps;
    gf::EdgeFillMode _edgeFillMode;
    double _fovScaleBoost;
    double _sharpenStrength;
    GFHistSlot _histSlots[3];
    GFStabilizerPlanObjC *_plan;
}

- (GFStabilizerPlanObjC *)plan {
    return _plan;
}

- (void)releaseHistoryBuffers {
    for (int i = 0; i < 3; ++i) {
        if (_histSlots[i].pb) {
            CFRelease(_histSlots[i].pb);
            _histSlots[i].pb = nullptr;
        }
        _histSlots[i].frameIndex = 0;
    }
}

- (void)dealloc {
    [self releaseHistoryBuffers];
    if (_textureCache) {
        CFRelease(_textureCache);
        _textureCache = nullptr;
    }
}

- (nullable instancetype)initWithSidecarPath:(NSString *)sidecarPath
                             pitchSmoothness:(double)pitchSmoothness
                               yawSmoothness:(double)yawSmoothness
                              rollSmoothness:(double)rollSmoothness
                             velocityDamping:(double)damping
                                timeOffsetMs:(double)offsetMs
                                maxZoomLimit:(double)maxZoomLimit
                          horizonLockEnabled:(BOOL)horizonLockEnabled
                           horizonLockAmount:(double)horizonLockAmount
                        horizonTiltOffsetDeg:(double)horizonTiltOffsetDeg
                    rollingShutterCorrection:(BOOL)rollingShutterCorrection
                          dynamicZoomEnabled:(BOOL)dynamicZoomEnabled
                             axisPresetIndex:(NSInteger)axisPresetIndex
                        videoRotationDegrees:(NSInteger)videoRotationDegrees
                       hasReadoutOverride:(BOOL)hasReadoutOverride
                      signedReadoutTimeMs:(double)signedReadoutTimeMs
                       focalScaleOverride:(double)focalScaleOverride
                       rollGyroScaleOverride:(double)rollGyroScaleOverride
                           motoActionMode:(BOOL)motoActionMode
                             edgeFillMode:(NSInteger)edgeFillMode
                            fovScaleBoost:(double)fovScaleBoost
                          sharpenStrength:(double)sharpenStrength {
    self = [super init];
    if (!self) {
        return nil;
    }

    _plan = [[GFStabilizerPlanObjC alloc] init];

    gf::GFFileHeader hdr{};
    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    std::vector<gf::GFFrameRecord> frames;
    gf::SidecarSummary summary;

    if (!gf::SidecarReader::readAll([sidecarPath UTF8String], hdr, gyro, accel, frames, summary) ||
        !summary.validHeader || frames.empty()) {
        _plan.valid = NO;
        _plan.errorMessage = [NSString stringWithUTF8String:summary.errorMessage.c_str()] ?: @"Invalid sidecar or empty frames";
        return self;
    }

    const double *customAxis = nullptr;
    const auto &presets = gf::OpticalSync::getAxisPresets();
    if (axisPresetIndex >= 0 && static_cast<size_t>(axisPresetIndex) < presets.size()) {
        customAxis = presets[static_cast<size_t>(axisPresetIndex)].m;
    }

    _engine.setImuData(hdr, gyro, accel, customAxis, motoActionMode ? true : false, rollGyroScaleOverride);

    _cfg = gf::StabilizerConfig{};
    _cfg.pitchSmoothness = pitchSmoothness;
    _cfg.yawSmoothness = yawSmoothness;
    _cfg.rollSmoothness = rollSmoothness;
    _cfg.velocityDamping = damping;
    _cfg.timeOffsetMs = offsetMs;
    _cfg.maxZoomLimit = maxZoomLimit;
    _cfg.horizonLockEnabled = horizonLockEnabled ? true : false;
    _cfg.horizonLockAmount = horizonLockAmount;
    _cfg.horizonTiltOffsetDeg = horizonTiltOffsetDeg;
    _cfg.rollingShutterCorrection = rollingShutterCorrection ? true : false;
    _cfg.dynamicZoomEnabled = dynamicZoomEnabled ? true : false;
    _cfg.motoActionMode = motoActionMode ? true : false;
    _cfg.videoRotationDegrees = (int)videoRotationDegrees;

    _traj = _engine.solve(frames, _cfg);
    if (!_traj.valid) {
        _plan.valid = NO;
        _plan.errorMessage = @"Trajectory solver failed.";
        return self;
    }

    _profile = (hdr.capture_mode == 1)
        ? gf::CalibrationProfile::defaultIPhone5s720p120()
        : gf::CalibrationProfile::defaultIPhone5s1080p30();
    if (hdr.width > 0 && hdr.height > 0 && hdr.lens_fx > 10.0) {
        _profile.width = hdr.width;
        _profile.height = hdr.height;
        _profile.fx = hdr.lens_fx;
        _profile.fy = hdr.lens_fy;
        _profile.cx = hdr.lens_cx;
        _profile.cy = hdr.lens_cy;
        _profile.k1 = hdr.dist_k1;
        _profile.k2 = hdr.dist_k2;
        _profile.p1 = hdr.dist_p1;
        _profile.p2 = hdr.dist_p2;
        _profile.readoutTimeSec = hdr.readout_time_sec;
        // Upgrade legacy over-strong Phase 2/3/4 default distortion coefficients & 1660px -> 1815px video FOV
        if (std::fabs(_profile.k1 - 0.042) < 1e-4 && std::fabs(_profile.k2 + 0.115) < 1e-4) {
            _profile.k1 = 0.012;
            _profile.k2 = -0.022;
        }
        if (std::fabs(_profile.fx - 1660.0) < 2.0 && _profile.width >= 1900) {
            _profile.fx = 1815.0;
            _profile.fy = 1815.0;
        } else if (std::fabs(_profile.fx - 1106.67) < 2.0 && _profile.width == 1280) {
            _profile.fx = 1210.0;
            _profile.fy = 1210.0;
        }
    }
    if (hasReadoutOverride) {
        _profile.readoutTimeSec = signedReadoutTimeMs * 0.001;
    }
    if (focalScaleOverride >= 0.85 && focalScaleOverride <= 1.20) {
        _profile.fx *= focalScaleOverride;
        _profile.fy *= focalScaleOverride;
    }

    _fps = (hdr.nominal_fps > 1.0) ? hdr.nominal_fps : 30.0;
    _sidecarFrames = frames;
    _edgeFillMode = (edgeFillMode == 1) ? gf::EdgeFillMode::TemporalFullFrame :
                    ((edgeFillMode == 2) ? gf::EdgeFillMode::MirrorEdge : gf::EdgeFillMode::StandardCrop);
    _fovScaleBoost = std::min(1.0, std::max(0.0, fovScaleBoost));
    _sharpenStrength = std::min(1.0, std::max(0.0, sharpenStrength));
    _hybridTracker.reset(_profile, _traj, _cfg.motoActionMode);

    const gf::WarpComputationResult warpRes = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        _engine,
        _traj,
        _profile,
        _fps,
        maxZoomLimit,
        _cfg.rollingShutterCorrection,
        _cfg.dynamicZoomEnabled,
        nullptr,
        nullptr,
        _edgeFillMode,
        _fovScaleBoost
    );

    _frameLUTs = std::move(warpRes.frameLUTs);
    _adjustedSmoothQuats = warpRes.adjustedSmoothQuats;
    _plan.frameCount = _frameLUTs.size();
    _plan.stabilityImprovementPct = _traj.stabilityImprovementPct;
    _plan.minZoom = warpRes.minZoom;
    _plan.maxZoom = warpRes.maxZoom;
    _plan.avgZoom = warpRes.avgZoom;
    _plan.maxInputRollDeg = _traj.maxInputRollDeg;
    _plan.maxOutputRollErrorDeg = _traj.maxOutputRollErrorDeg;
    _plan.maxCorrectionDeg = _traj.maxCorrectionDeg;
    const int quadDeg = _traj.detectedGravityQuadrantDeg;
    if (quadDeg == 90 || quadDeg == 270) {
        _plan.detectedOrientationName = [NSString stringWithFormat:@"Auto-Portrait (%d°)", quadDeg];
    } else {
        _plan.detectedOrientationName = [NSString stringWithFormat:@"Auto-Landscape (%d°)", quadDeg];
    }
    if (!_frameLUTs.empty()) {
        const size_t midIdx = _frameLUTs.size() / 2;
        const float *m = _frameLUTs[midIdx].sliceH[gf::kRollingShutterSlices / 2];
        _plan.isIdentityMatrix = (std::fabs(m[0] - 1.0f) < 1e-3f &&
                                  std::fabs(m[4] - 1.0f) < 1e-3f &&
                                  std::fabs(m[1]) < 1e-3f &&
                                  std::fabs(m[3]) < 1e-3f) ? YES : NO;
        _plan.gpuMatrixProof = [NSString stringWithFormat:
            @"MidFrame H: diag=(%.3f, %.3f) | rot=(%.3f, %.3f) | inputTilt=%.1f° -> outErr=%.2f°",
            m[0], m[4], m[1], m[3], _traj.maxInputRollDeg, _traj.maxOutputRollErrorDeg];
    }
    _plan.width = _profile.width;
    _plan.height = _profile.height;
    _plan.nominalFps = _fps;

    _device = MTLCreateSystemDefaultDevice();
    if (!_device) {
        _plan.valid = NO;
        _plan.errorMessage = @"Metal device unavailable.";
        return self;
    }

    _commandQueue = [_device newCommandQueue];
    id<MTLLibrary> library = [_device newDefaultLibrary];
    if (!_commandQueue || !library) {
        _plan.valid = NO;
        _plan.errorMessage = @"Failed to create Metal command queue or default library.";
        return self;
    }

    NSError *err = nil;
    id<MTLFunction> lumaFn = [library newFunctionWithName:@"lanczos4_warp_luma"];
    id<MTLFunction> chromaFn = [library newFunctionWithName:@"lanczos4_warp_chroma"];
    if (!lumaFn || !chromaFn) {
        _plan.valid = NO;
        _plan.errorMessage = @"Missing Metal warp kernel functions.";
        return self;
    }

    _lumaPSO = [_device newComputePipelineStateWithFunction:lumaFn error:&err];
    if (!_lumaPSO || err) {
        _plan.valid = NO;
        _plan.errorMessage = err.localizedDescription ?: @"Failed to compile luma PSO.";
        return self;
    }

    _chromaPSO = [_device newComputePipelineStateWithFunction:chromaFn error:&err];
    if (!_chromaPSO || err) {
        _plan.valid = NO;
        _plan.errorMessage = err.localizedDescription ?: @"Failed to compile chroma PSO.";
        return self;
    }

    CVReturn cacheStatus = CVMetalTextureCacheCreate(kCFAllocatorDefault, nullptr, _device, nullptr, &_textureCache);
    if (cacheStatus != kCVReturnSuccess || !_textureCache) {
        _plan.valid = NO;
        _plan.errorMessage = @"Failed to create CVMetalTextureCache.";
        return self;
    }

    _plan.valid = YES;
    return self;
}

- (void)pushHybridResidualPixelBuffer:(CVPixelBufferRef)pixelBuffer timestampSec:(double)ts {
    if (!pixelBuffer) {
        return;
    }
    CVPixelBufferLockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
    const size_t srcW = CVPixelBufferGetWidthOfPlane(pixelBuffer, 0);
    const size_t srcH = CVPixelBufferGetHeightOfPlane(pixelBuffer, 0);
    const size_t stride = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0);
    const uint8_t *base = (const uint8_t *)CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0);

    if (base && srcW >= 64 && srcH >= 64) {
        const int ds = 4;
        const int dstW = (int)(srcW / ds);
        const int dstH = (int)(srcH / ds);
        gf::OpticalFlowFrame f;
        f.timestampSec = ts;
        f.width = dstW;
        f.height = dstH;
        f.luma.resize((size_t)(dstW * dstH));

        for (int y = 0; y < dstH; ++y) {
            const uint8_t *srcRow = base + (size_t)(y * ds) * stride;
            uint8_t *dstRow = f.luma.data() + (size_t)(y * dstW);
            for (int x = 0; x < dstW; ++x) {
                dstRow[x] = srcRow[x * ds];
            }
        }
        _hybridTracker.pushFrame(f);
    }
    CVPixelBufferUnlockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
}

- (void)finalizeHybridResidual {
    const gf::HybridResidualResult hr = _hybridTracker.finalize(_fps);
    if (!hr.valid) {
        return;
    }
    _residualShiftX = hr.shiftX;
    _residualShiftY = hr.shiftY;
    const gf::WarpComputationResult warpRes = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        _engine,
        _traj,
        _profile,
        _fps,
        _cfg.maxZoomLimit,
        _cfg.rollingShutterCorrection,
        _cfg.dynamicZoomEnabled,
        &_residualShiftX,
        &_residualShiftY,
        _edgeFillMode,
        _fovScaleBoost
    );
    _frameLUTs = std::move(warpRes.frameLUTs);
    _adjustedSmoothQuats = warpRes.adjustedSmoothQuats;
    _plan.minZoom = warpRes.minZoom;
    _plan.maxZoom = warpRes.maxZoom;
    _plan.avgZoom = warpRes.avgZoom;
    _plan.hybridResidualMaxPx = hr.maxAmplitudePx;
}

- (BOOL)warpPixelBuffer:(CVPixelBufferRef)srcPixelBuffer
        intoPixelBuffer:(CVPixelBufferRef)dstPixelBuffer
             frameIndex:(NSUInteger)frameIndex {
    if (!_plan.valid || _frameLUTs.empty() || !_textureCache) {
        return NO;
    }

    const NSUInteger clampedIdx = (frameIndex < _frameLUTs.size()) ? frameIndex : (_frameLUTs.size() - 1);
    const gf::GFWarpFrameLUT &lut = _frameLUTs[clampedIdx];

    const size_t srcW = CVPixelBufferGetWidth(srcPixelBuffer);
    const size_t srcH = CVPixelBufferGetHeight(srcPixelBuffer);
    const size_t dstW = CVPixelBufferGetWidth(dstPixelBuffer);
    const size_t dstH = CVPixelBufferGetHeight(dstPixelBuffer);

    CVMetalTextureRef srcYRef = nullptr;
    CVMetalTextureRef srcUVRef = nullptr;
    CVMetalTextureRef dstYRef = nullptr;
    CVMetalTextureRef dstUVRef = nullptr;

    CVReturn sY = CVMetalTextureCacheCreateTextureFromImage(
        kCFAllocatorDefault, _textureCache, srcPixelBuffer, nullptr,
        MTLPixelFormatR8Unorm, srcW, srcH, 0, &srcYRef
    );
    CVReturn sUV = CVMetalTextureCacheCreateTextureFromImage(
        kCFAllocatorDefault, _textureCache, srcPixelBuffer, nullptr,
        MTLPixelFormatRG8Unorm, srcW / 2, srcH / 2, 1, &srcUVRef
    );
    CVReturn dY = CVMetalTextureCacheCreateTextureFromImage(
        kCFAllocatorDefault, _textureCache, dstPixelBuffer, nullptr,
        MTLPixelFormatR8Unorm, dstW, dstH, 0, &dstYRef
    );
    CVReturn dUV = CVMetalTextureCacheCreateTextureFromImage(
        kCFAllocatorDefault, _textureCache, dstPixelBuffer, nullptr,
        MTLPixelFormatRG8Unorm, dstW / 2, dstH / 2, 1, &dstUVRef
    );

    if (sY != kCVReturnSuccess || sUV != kCVReturnSuccess ||
        dY != kCVReturnSuccess || dUV != kCVReturnSuccess ||
        !srcYRef || !srcUVRef || !dstYRef || !dstUVRef) {
        if (srcYRef) CFRelease(srcYRef);
        if (srcUVRef) CFRelease(srcUVRef);
        if (dstYRef) CFRelease(dstYRef);
        if (dstUVRef) CFRelease(dstUVRef);
        return NO;
    }

    id<MTLTexture> srcYTex = CVMetalTextureGetTexture(srcYRef);
    id<MTLTexture> srcUVTex = CVMetalTextureGetTexture(srcUVRef);
    id<MTLTexture> dstYTex = CVMetalTextureGetTexture(dstYRef);
    id<MTLTexture> dstUVTex = CVMetalTextureGetTexture(dstUVRef);

    id<MTLCommandBuffer> cmdBuf = [_commandQueue commandBuffer];
    id<MTLComputeCommandEncoder> enc = [cmdBuf computeCommandEncoder];

    GFWarpFrameUniformsMetal lumaU{};
    for (int s = 0; s < gf::kRollingShutterSlices; ++s) {
        lumaU.slices[s].hRow0 = simd_make_float4(lut.sliceH[s][0], lut.sliceH[s][1], lut.sliceH[s][2], 0.0f);
        lumaU.slices[s].hRow1 = simd_make_float4(lut.sliceH[s][3], lut.sliceH[s][4], lut.sliceH[s][5], 0.0f);
        lumaU.slices[s].hRow2 = simd_make_float4(lut.sliceH[s][6], lut.sliceH[s][7], lut.sliceH[s][8], 0.0f);
    }

    int activeHistCount = 0;
    float gains[3] = {1.0f, 1.0f, 1.0f};
    CVMetalTextureRef histYRefs[3] = {nullptr, nullptr, nullptr};
    CVMetalTextureRef histUVRefs[3] = {nullptr, nullptr, nullptr};
    id<MTLTexture> histYTex[3] = {srcYTex, srcYTex, srcYTex};
    id<MTLTexture> histUVTex[3] = {srcUVTex, srcUVTex, srcUVTex};

    if (_edgeFillMode == gf::EdgeFillMode::TemporalFullFrame) {
        const gf::Quat qTarget = (clampedIdx < _adjustedSmoothQuats.size())
            ? _adjustedSmoothQuats[clampedIdx]
            : ((clampedIdx < _traj.smoothFrameQuats.size()) ? _traj.smoothFrameQuats[clampedIdx] : gf::Quat::identity());
        const double sxOff = (clampedIdx < _residualShiftX.size()) ? _residualShiftX[clampedIdx] : 0.0;
        const double syOff = (clampedIdx < _residualShiftY.size()) ? _residualShiftY[clampedIdx] : 0.0;

        float curExpIso = 1.0f;
        if (clampedIdx < _sidecarFrames.size()) {
            curExpIso = std::max(1e-6f, _sidecarFrames[clampedIdx].exposure_duration_sec * std::max(1.0f, _sidecarFrames[clampedIdx].iso));
        }

        for (int h = 0; h < 3; ++h) {
            if (!_histSlots[h].pb) {
                continue;
            }
            const NSUInteger hIdx = (_histSlots[h].frameIndex < _traj.rawFrameQuats.size())
                ? _histSlots[h].frameIndex
                : (_traj.rawFrameQuats.size() - 1);
            const size_t hw = CVPixelBufferGetWidth(_histSlots[h].pb);
            const size_t hh = CVPixelBufferGetHeight(_histSlots[h].pb);
            if (hw != srcW || hh != srcH) {
                continue;
            }
            CVReturn rY = CVMetalTextureCacheCreateTextureFromImage(
                kCFAllocatorDefault, _textureCache, _histSlots[h].pb, nullptr,
                MTLPixelFormatR8Unorm, hw, hh, 0, &histYRefs[activeHistCount]
            );
            CVReturn rUV = CVMetalTextureCacheCreateTextureFromImage(
                kCFAllocatorDefault, _textureCache, _histSlots[h].pb, nullptr,
                MTLPixelFormatRG8Unorm, hw / 2, hh / 2, 1, &histUVRefs[activeHistCount]
            );
            if (rY == kCVReturnSuccess && rUV == kCVReturnSuccess && histYRefs[activeHistCount] && histUVRefs[activeHistCount]) {
                histYTex[activeHistCount] = CVMetalTextureGetTexture(histYRefs[activeHistCount]);
                histUVTex[activeHistCount] = CVMetalTextureGetTexture(histUVRefs[activeHistCount]);

                const gf::Quat qRawHist = _traj.rawFrameQuats[hIdx];
                const gf::Quat qHistCorr = (qRawHist.conjugate() * qTarget).normalized();
                float hMat[9];
                gf::FrameWarpCalculator::computeInverseHomography(
                    qHistCorr, _profile.fx, _profile.fy, _profile.cx, _profile.cy,
                    (double)lut.zoomUsed, hMat, sxOff, syOff
                );
                lumaU.histH[activeHistCount].hRow0 = simd_make_float4(hMat[0], hMat[1], hMat[2], 0.0f);
                lumaU.histH[activeHistCount].hRow1 = simd_make_float4(hMat[3], hMat[4], hMat[5], 0.0f);
                lumaU.histH[activeHistCount].hRow2 = simd_make_float4(hMat[6], hMat[7], hMat[8], 0.0f);

                if (hIdx < _sidecarFrames.size()) {
                    const float hExpIso = std::max(1e-6f, _sidecarFrames[hIdx].exposure_duration_sec * std::max(1.0f, _sidecarFrames[hIdx].iso));
                    gains[activeHistCount] = std::min(1.35f, std::max(0.75f, curExpIso / hExpIso));
                }
                ++activeHistCount;
            } else {
                if (histYRefs[activeHistCount]) { CFRelease(histYRefs[activeHistCount]); histYRefs[activeHistCount] = nullptr; }
                if (histUVRefs[activeHistCount]) { CFRelease(histUVRefs[activeHistCount]); histUVRefs[activeHistCount] = nullptr; }
            }
        }
    }

    const float readoutAxis = (srcH > srcW) ? 1.0f : 0.0f;
    lumaU.histGains = simd_make_float4(gains[0], gains[1], gains[2], readoutAxis);
    lumaU.polishParams = simd_make_float4((float)(int)_edgeFillMode, (float)_sharpenStrength, lut.zoomUsed, (float)activeHistCount);
    lumaU.outSize = simd_make_uint2((uint32_t)dstW, (uint32_t)dstH);
    lumaU.inSize = simd_make_uint2((uint32_t)srcW, (uint32_t)srcH);
    lumaU.lumaOutSize = simd_make_uint2((uint32_t)dstW, (uint32_t)dstH);
    lumaU.lumaInSize = simd_make_uint2((uint32_t)srcW, (uint32_t)srcH);
    lumaU.intrinsics = simd_make_float4((float)_profile.fx, (float)_profile.fy, (float)_profile.cx, (float)_profile.cy);
    lumaU.distortion = simd_make_float4((float)_profile.k1, (float)_profile.k2, (float)_profile.p1, (float)_profile.p2);

    const MTLSize tgSize = MTLSizeMake(16, 16, 1);
    const MTLSize lumaGroups = MTLSizeMake((dstW + 15) / 16, (dstH + 15) / 16, 1);

    [enc setComputePipelineState:_lumaPSO];
    [enc setTexture:srcYTex atIndex:0];
    [enc setTexture:dstYTex atIndex:1];
    [enc setTexture:histYTex[0] atIndex:2];
    [enc setTexture:histYTex[1] atIndex:3];
    [enc setTexture:histYTex[2] atIndex:4];
    [enc setBytes:&lumaU length:sizeof(GFWarpFrameUniformsMetal) atIndex:0];
    [enc dispatchThreadgroups:lumaGroups threadsPerThreadgroup:tgSize];

    GFWarpFrameUniformsMetal chromaU = lumaU;
    chromaU.outSize = simd_make_uint2((uint32_t)(dstW / 2), (uint32_t)(dstH / 2));
    chromaU.inSize = simd_make_uint2((uint32_t)(srcW / 2), (uint32_t)(srcH / 2));

    const MTLSize chromaGroups = MTLSizeMake(((dstW / 2) + 15) / 16, ((dstH / 2) + 15) / 16, 1);

    [enc setComputePipelineState:_chromaPSO];
    [enc setTexture:srcUVTex atIndex:0];
    [enc setTexture:dstUVTex atIndex:1];
    [enc setTexture:histUVTex[0] atIndex:2];
    [enc setTexture:histUVTex[1] atIndex:3];
    [enc setTexture:histUVTex[2] atIndex:4];
    [enc setBytes:&chromaU length:sizeof(GFWarpFrameUniformsMetal) atIndex:0];
    [enc dispatchThreadgroups:chromaGroups threadsPerThreadgroup:tgSize];

    [enc endEncoding];
    [cmdBuf commit];
    [cmdBuf waitUntilCompleted];

    for (int i = 0; i < 3; ++i) {
        histYTex[i] = nil;
        histUVTex[i] = nil;
        if (histYRefs[i]) CFRelease(histYRefs[i]);
        if (histUVRefs[i]) CFRelease(histUVRefs[i]);
    }
    srcYTex = nil;
    srcUVTex = nil;
    dstYTex = nil;
    dstUVTex = nil;

    CFRelease(srcYRef);
    srcYRef = nullptr;
    CFRelease(srcUVRef);
    srcUVRef = nullptr;
    CFRelease(dstYRef);
    dstYRef = nullptr;
    CFRelease(dstUVRef);
    dstUVRef = nullptr;

    // Update 3-frame temporal ring buffer (slot 0: i-1, slot 1: ~i-4, slot 2: ~i-8)
    if (_edgeFillMode == gf::EdgeFillMode::TemporalFullFrame) {
        if (_histSlots[0].pb && (!_histSlots[1].pb || (frameIndex >= _histSlots[1].frameIndex + 4))) {
            if (_histSlots[1].pb && (!_histSlots[2].pb || (frameIndex >= _histSlots[2].frameIndex + 8))) {
                if (_histSlots[2].pb) CFRelease(_histSlots[2].pb);
                _histSlots[2] = _histSlots[1];
                _histSlots[1] = _histSlots[0];
                _histSlots[0].pb = nullptr;
            } else {
                if (_histSlots[1].pb) CFRelease(_histSlots[1].pb);
                _histSlots[1] = _histSlots[0];
                _histSlots[0].pb = nullptr;
            }
        }
        if (_histSlots[0].pb) {
            CFRelease(_histSlots[0].pb);
        }
        CFRetain(srcPixelBuffer);
        _histSlots[0].pb = srcPixelBuffer;
        _histSlots[0].frameIndex = frameIndex;
    }

    CVMetalTextureCacheFlush(_textureCache, 0);

    return (cmdBuf.status == MTLCommandBufferStatusCompleted);
}

- (void)flushTextureCache {
    [self releaseHistoryBuffers];
    if (_textureCache) {
        CVMetalTextureCacheFlush(_textureCache, 0);
    }
}

@end
