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
    GFMtlRowSlice slices[16];
    simd_uint2 outSize;
    simd_uint2 inSize;
    simd_uint2 lumaOutSize;
    simd_uint2 lumaInSize;
    simd_float4 intrinsics;
    simd_float4 distortion;
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

- (GFOpticalSyncResultObjC *)solveWithSidecarPath:(NSString *)sidecarPath {
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

    // Align OpticalFlowFrame timestamps with sidecar frame PTS if needed
    const size_t count = std::min(_frames.size(), scFrames.size());
    if (count > 0 && !_frames.empty()) {
        // Use sidecar's Mach-domain PTS when frame count matches or offset from start
        const double pts0 = scFrames[0].pts_sec;
        const double vid0 = _frames[0].timestampSec;
        const double delta = pts0 - vid0;
        for (auto &f : _frames) {
            f.timestampSec += delta;
        }
    }

    const gf::OpticalSyncResult res = gf::OpticalSync::solveSyncAndAxes(_frames, hdr, gyro);
    out.valid = res.valid ? YES : NO;
    out.bestOffsetMs = res.bestOffsetMs;
    out.bestNCC = res.bestNCC;
    out.bestAxisPresetIndex = res.bestAxisPresetIndex;
    out.axisPresetName = [NSString stringWithUTF8String:res.axisPresetName.c_str()] ?: @"";
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
    gf::CalibrationProfile _profile;
    GFStabilizerPlanObjC *_plan;
}

- (GFStabilizerPlanObjC *)plan {
    return _plan;
}

- (void)dealloc {
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
                        videoRotationDegrees:(NSInteger)videoRotationDegrees {
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

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel, customAxis);

    gf::StabilizerConfig cfg;
    cfg.pitchSmoothness = pitchSmoothness;
    cfg.yawSmoothness = yawSmoothness;
    cfg.rollSmoothness = rollSmoothness;
    cfg.velocityDamping = damping;
    cfg.timeOffsetMs = offsetMs;
    cfg.maxZoomLimit = maxZoomLimit;
    cfg.horizonLockEnabled = horizonLockEnabled ? true : false;
    cfg.horizonLockAmount = horizonLockAmount;
    cfg.horizonTiltOffsetDeg = horizonTiltOffsetDeg;
    cfg.rollingShutterCorrection = rollingShutterCorrection ? true : false;
    cfg.dynamicZoomEnabled = dynamicZoomEnabled ? true : false;
    cfg.videoRotationDegrees = (int)videoRotationDegrees;

    const gf::TrajectoryResult traj = engine.solve(frames, cfg);
    if (!traj.valid) {
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
    }

    const double fps = (hdr.nominal_fps > 1.0) ? hdr.nominal_fps : 30.0;
    const gf::WarpComputationResult warpRes = gf::FrameWarpCalculator::computeAllFramesWithRollingShutter(
        engine,
        traj,
        _profile,
        fps,
        maxZoomLimit,
        cfg.rollingShutterCorrection,
        cfg.dynamicZoomEnabled
    );

    _frameLUTs = std::move(warpRes.frameLUTs);
    _plan.frameCount = _frameLUTs.size();
    _plan.stabilityImprovementPct = traj.stabilityImprovementPct;
    _plan.minZoom = warpRes.minZoom;
    _plan.maxZoom = warpRes.maxZoom;
    _plan.avgZoom = warpRes.avgZoom;
    _plan.maxInputRollDeg = traj.maxInputRollDeg;
    _plan.maxOutputRollErrorDeg = traj.maxOutputRollErrorDeg;
    _plan.maxCorrectionDeg = traj.maxCorrectionDeg;
    const int quadDeg = traj.detectedGravityQuadrantDeg;
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
            m[0], m[4], m[1], m[3], traj.maxInputRollDeg, traj.maxOutputRollErrorDeg];
    }
    _plan.width = _profile.width;
    _plan.height = _profile.height;
    _plan.nominalFps = fps;

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
    for (int s = 0; s < 16; ++s) {
        lumaU.slices[s].hRow0 = simd_make_float4(lut.sliceH[s][0], lut.sliceH[s][1], lut.sliceH[s][2], 0.0f);
        lumaU.slices[s].hRow1 = simd_make_float4(lut.sliceH[s][3], lut.sliceH[s][4], lut.sliceH[s][5], 0.0f);
        lumaU.slices[s].hRow2 = simd_make_float4(lut.sliceH[s][6], lut.sliceH[s][7], lut.sliceH[s][8], 0.0f);
    }
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
    [enc setBytes:&lumaU length:sizeof(GFWarpFrameUniformsMetal) atIndex:0];
    [enc dispatchThreadgroups:lumaGroups threadsPerThreadgroup:tgSize];

    GFWarpFrameUniformsMetal chromaU = lumaU;
    chromaU.outSize = simd_make_uint2((uint32_t)(dstW / 2), (uint32_t)(dstH / 2));
    chromaU.inSize = simd_make_uint2((uint32_t)(srcW / 2), (uint32_t)(srcH / 2));

    const MTLSize chromaGroups = MTLSizeMake(((dstW / 2) + 15) / 16, ((dstH / 2) + 15) / 16, 1);

    [enc setComputePipelineState:_chromaPSO];
    [enc setTexture:srcUVTex atIndex:0];
    [enc setTexture:dstUVTex atIndex:1];
    [enc setBytes:&chromaU length:sizeof(GFWarpFrameUniformsMetal) atIndex:0];
    [enc dispatchThreadgroups:chromaGroups threadsPerThreadgroup:tgSize];

    [enc endEncoding];
    [cmdBuf commit];
    [cmdBuf waitUntilCompleted];

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

    CVMetalTextureCacheFlush(_textureCache, 0);

    return (cmdBuf.status == MTLCommandBufferStatusCompleted);
}

- (void)flushTextureCache {
    if (_textureCache) {
        CVMetalTextureCacheFlush(_textureCache, 0);
    }
}

@end
