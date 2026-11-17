#import "SystemProbeBridge.h"
#import "Quaternion.hpp"

#import <AVFoundation/AVFoundation.h>
#import <CoreVideo/CoreVideo.h>
#import <Metal/Metal.h>
#import <dispatch/dispatch.h>
#import <mach/mach.h>
#import <mach/mach_time.h>

#include <cmath>

@implementation GFSystemProbe

+ (uint64_t)currentPhysFootprintBytes {
    task_vm_info_data_t vmInfo;
    mach_msg_type_number_t count = TASK_VM_INFO_COUNT;
    kern_return_t kr = task_info(mach_task_self(), TASK_VM_INFO, (task_info_t)&vmInfo, &count);
    if (kr != KERN_SUCCESS) {
        return 0;
    }
    return (uint64_t)vmInfo.phys_footprint;
}

+ (double)currentMachBootTimeSeconds {
    static mach_timebase_info_data_t timebaseInfo;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
        mach_timebase_info(&timebaseInfo);
    });
    const uint64_t now = mach_absolute_time();
    const double nanos = (double)now * (double)timebaseInfo.numer / (double)timebaseInfo.denom;
    return nanos / 1e9;
}

+ (NSString *)verifyCppCoreSelfTest {
    const gf::Vec3 gravity = gf::Vec3(0.18, -0.92, 0.35).normalized();
    const gf::Quat tilt = gf::Quat::fromAxisAngle({0.7, -0.4, 0.2}, 0.48);
    const gf::Quat yaw = gf::Quat::fromAxisAngle(gravity, 1.35);
    const gf::Quat q = (tilt * yaw).normalized();

    gf::Quat swing;
    gf::Quat twist;
    gf::Quat::decomposeSwingTwist(q, gravity, swing, twist);

    const gf::Quat recon = (swing * twist).normalized();
    const double reconErr = std::fabs(recon.w - q.w) + std::fabs(recon.x - q.x) +
                            std::fabs(recon.y - q.y) + std::fabs(recon.z - q.z);
    const gf::Vec3 rotG = twist.rotateVector(gravity);
    const double axisErr = (rotG - gravity).norm();

    const BOOL passed = (reconErr < 1e-11) && (axisErr < 1e-11);
    return [NSString stringWithFormat:@"%@ (reconErr=%.2e, axisErr=%.2e)",
            passed ? @"PASS" : @"FAIL", reconErr, axisErr];
}

+ (NSDictionary<NSString *, id> *)probeMetalA7Capabilities {
    NSMutableDictionary<NSString *, id> *result = [NSMutableDictionary dictionary];

    id<MTLDevice> device = MTLCreateSystemDefaultDevice();
    if (!device) {
        result[@"error"] = @"MTLCreateSystemDefaultDevice returned nil";
        return result;
    }

    result[@"deviceName"] = device.name ?: @"Unknown";

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
    const BOOL supportsGPU1v5 = [device supportsFeatureSet:MTLFeatureSet_iOS_GPUFamily1_v5];
#pragma clang diagnostic pop
    result[@"supportsGPUFamily1v5"] = @(supportsGPU1v5);

    CVMetalTextureCacheRef textureCache = nullptr;
    CVReturn cacheStatus = CVMetalTextureCacheCreate(kCFAllocatorDefault, nullptr, device, nullptr, &textureCache);
    result[@"textureCacheCreated"] = @(cacheStatus == kCVReturnSuccess && textureCache != nullptr);

    CVPixelBufferRef pixelBuffer = nullptr;
    NSDictionary *pbAttrs = @{
        (id)kCVPixelBufferMetalCompatibilityKey: @YES,
        (id)kCVPixelBufferIOSurfacePropertiesKey: @{}
    };
    CVReturn pbStatus = CVPixelBufferCreate(
        kCFAllocatorDefault,
        1920,
        1080,
        kCVPixelFormatType_420YpCbCr8BiPlanarFullRange,
        (__bridge CFDictionaryRef)pbAttrs,
        &pixelBuffer
    );
    result[@"pixelBufferCreated"] = @(pbStatus == kCVReturnSuccess && pixelBuffer != nullptr);

    BOOL lumaMapped = NO;
    BOOL chromaMapped = NO;

    if (textureCache && pixelBuffer) {
        CVMetalTextureRef lumaTexRef = nullptr;
        CVReturn lumaStatus = CVMetalTextureCacheCreateTextureFromImage(
            kCFAllocatorDefault,
            textureCache,
            pixelBuffer,
            nullptr,
            MTLPixelFormatR8Unorm,
            1920,
            1080,
            0,
            &lumaTexRef
        );
        if (lumaStatus == kCVReturnSuccess && lumaTexRef) {
            id<MTLTexture> lumaTex = CVMetalTextureGetTexture(lumaTexRef);
            lumaMapped = (lumaTex != nil && lumaTex.width == 1920 && lumaTex.height == 1080);
            CFRelease(lumaTexRef);
        }

        CVMetalTextureRef chromaTexRef = nullptr;
        CVReturn chromaStatus = CVMetalTextureCacheCreateTextureFromImage(
            kCFAllocatorDefault,
            textureCache,
            pixelBuffer,
            nullptr,
            MTLPixelFormatRG8Unorm,
            960,
            540,
            1,
            &chromaTexRef
        );
        if (chromaStatus == kCVReturnSuccess && chromaTexRef) {
            id<MTLTexture> chromaTex = CVMetalTextureGetTexture(chromaTexRef);
            chromaMapped = (chromaTex != nil && chromaTex.width == 960 && chromaTex.height == 540);
            CFRelease(chromaTexRef);
        }
    }

    result[@"lumaTextureMapped"] = @(lumaMapped);
    result[@"chromaTextureMapped"] = @(chromaMapped);

    if (pixelBuffer) {
        CFRelease(pixelBuffer);
    }
    if (textureCache) {
        CFRelease(textureCache);
    }

    id<MTLLibrary> library = [device newDefaultLibrary];
    result[@"defaultLibraryLoaded"] = @(library != nil);

    BOOL lumaPipelineOK = NO;
    BOOL chromaPipelineOK = NO;
    NSUInteger maxThreads = 0;

    if (library) {
        NSError *err = nil;
        id<MTLFunction> lumaFn = [library newFunctionWithName:@"lanczos4_warp_luma"];
        if (lumaFn) {
            id<MTLComputePipelineState> pso = [device newComputePipelineStateWithFunction:lumaFn error:&err];
            if (pso && !err) {
                lumaPipelineOK = YES;
                maxThreads = pso.maxTotalThreadsPerThreadgroup;
            } else if (err) {
                result[@"lumaPipelineError"] = err.localizedDescription;
            }
        }

        err = nil;
        id<MTLFunction> chromaFn = [library newFunctionWithName:@"lanczos4_warp_chroma"];
        if (chromaFn) {
            id<MTLComputePipelineState> pso = [device newComputePipelineStateWithFunction:chromaFn error:&err];
            if (pso && !err) {
                chromaPipelineOK = YES;
            } else if (err) {
                result[@"chromaPipelineError"] = err.localizedDescription;
            }
        }
    }

    result[@"lumaPipelineCompiled"] = @(lumaPipelineOK);
    result[@"chromaPipelineCompiled"] = @(chromaPipelineOK);
    result[@"maxThreadsPerThreadgroup"] = @(maxThreads);

    return result;
}

@end
