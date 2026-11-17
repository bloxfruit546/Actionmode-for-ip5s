#import "StabilizerBridge.h"
#import "CalibrationProfile.hpp"
#import "FrameWarpCalculator.hpp"
#import "SidecarIO.hpp"
#import "TrajectoryEngine.hpp"

#import <Metal/Metal.h>
#include <simd/simd.h>
#include <vector>

namespace {

struct GFWarpUniformsMetal {
    simd_float4 hRow0;
    simd_float4 hRow1;
    simd_float4 hRow2;
    simd_uint2 outSize;
    simd_uint2 inSize;
};

} // namespace

@implementation GFStabilizerPlanObjC
- (instancetype)init {
    self = [super init];
    if (self) {
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
    std::vector<gf::GFWarpMatrixPacket> _packets;
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
                                  smoothness:(double)smoothness
                             velocityDamping:(double)damping
                                timeOffsetMs:(double)offsetMs
                                maxZoomLimit:(double)maxZoomLimit {
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

    gf::TrajectoryEngine engine;
    engine.setImuData(hdr, gyro, accel);

    gf::StabilizerConfig cfg;
    cfg.smoothness = smoothness;
    cfg.velocityDamping = damping;
    cfg.timeOffsetMs = offsetMs;
    cfg.maxZoomLimit = maxZoomLimit;

    const gf::TrajectoryResult traj = engine.solve(frames, cfg);
    if (!traj.valid) {
        _plan.valid = NO;
        _plan.errorMessage = @"Trajectory solver failed.";
        return self;
    }

    gf::CalibrationProfile prof = (hdr.capture_mode == 1)
        ? gf::CalibrationProfile::defaultIPhone5s720p120()
        : gf::CalibrationProfile::defaultIPhone5s1080p30();
    if (hdr.width > 0 && hdr.height > 0 && hdr.lens_fx > 10.0) {
        prof.width = hdr.width;
        prof.height = hdr.height;
        prof.fx = hdr.lens_fx;
        prof.fy = hdr.lens_fy;
        prof.cx = hdr.lens_cx;
        prof.cy = hdr.lens_cy;
        prof.k1 = hdr.dist_k1;
        prof.k2 = hdr.dist_k2;
        prof.p1 = hdr.dist_p1;
        prof.p2 = hdr.dist_p2;
        prof.readoutTimeSec = hdr.readout_time_sec;
    }

    const double fps = (hdr.nominal_fps > 1.0) ? hdr.nominal_fps : 30.0;
    const gf::WarpComputationResult warpRes = gf::FrameWarpCalculator::computeAllFrames(
        traj.rawFrameQuats,
        traj.smoothFrameQuats,
        prof,
        fps,
        maxZoomLimit
    );

    _packets = std::move(warpRes.packets);
    _plan.frameCount = _packets.size();
    _plan.stabilityImprovementPct = traj.stabilityImprovementPct;
    _plan.minZoom = warpRes.minZoom;
    _plan.maxZoom = warpRes.maxZoom;
    _plan.avgZoom = warpRes.avgZoom;
    _plan.width = prof.width;
    _plan.height = prof.height;
    _plan.nominalFps = fps;

    // Initialize Metal A7 compute pipelines
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
    if (!_plan.valid || _packets.empty() || !_textureCache) {
        return NO;
    }

    const NSUInteger clampedIdx = (frameIndex < _packets.size()) ? frameIndex : (_packets.size() - 1);
    const gf::GFWarpMatrixPacket &pkt = _packets[clampedIdx];

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

    // 1. Luma pass (16x16 = 256 threads/group, safe for A7 max 512)
    GFWarpUniformsMetal lumaU{};
    lumaU.hRow0 = simd_make_float4(pkt.m[0], pkt.m[1], pkt.m[2], 0.0f);
    lumaU.hRow1 = simd_make_float4(pkt.m[3], pkt.m[4], pkt.m[5], 0.0f);
    lumaU.hRow2 = simd_make_float4(pkt.m[6], pkt.m[7], pkt.m[8], 0.0f);
    lumaU.outSize = simd_make_uint2((uint32_t)dstW, (uint32_t)dstH);
    lumaU.inSize = simd_make_uint2((uint32_t)srcW, (uint32_t)srcH);

    const MTLSize tgSize = MTLSizeMake(16, 16, 1);
    const MTLSize lumaGroups = MTLSizeMake((dstW + 15) / 16, (dstH + 15) / 16, 1);

    [enc setComputePipelineState:_lumaPSO];
    [enc setTexture:srcYTex atIndex:0];
    [enc setTexture:dstYTex atIndex:1];
    [enc setBytes:&lumaU length:sizeof(GFWarpUniformsMetal) atIndex:0];
    [enc dispatchThreadgroups:lumaGroups threadsPerThreadgroup:tgSize];

    // 2. Chroma pass (half-resolution)
    GFWarpUniformsMetal chromaU = lumaU;
    chromaU.outSize = simd_make_uint2((uint32_t)(dstW / 2), (uint32_t)(dstH / 2));
    chromaU.inSize = simd_make_uint2((uint32_t)(srcW / 2), (uint32_t)(srcH / 2));

    const MTLSize chromaGroups = MTLSizeMake(((dstW / 2) + 15) / 16, ((dstH / 2) + 15) / 16, 1);

    [enc setComputePipelineState:_chromaPSO];
    [enc setTexture:srcUVTex atIndex:0];
    [enc setTexture:dstUVTex atIndex:1];
    [enc setBytes:&chromaU length:sizeof(GFWarpUniformsMetal) atIndex:0];
    [enc dispatchThreadgroups:chromaGroups threadsPerThreadgroup:tgSize];

    [enc endEncoding];
    [cmdBuf commit];
    [cmdBuf waitUntilCompleted];

    CFRelease(srcYRef);
    CFRelease(srcUVRef);
    CFRelease(dstYRef);
    CFRelease(dstUVRef);

    return (cmdBuf.status == MTLCommandBufferStatusCompleted);
}

- (void)flushTextureCache {
    if (_textureCache) {
        CVMetalTextureCacheFlush(_textureCache, 0);
    }
}

@end
