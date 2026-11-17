#import <CoreVideo/CoreVideo.h>
#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@interface GFStabilizerPlanObjC : NSObject
@property (nonatomic, assign) BOOL valid;
@property (nonatomic, assign) NSUInteger frameCount;
@property (nonatomic, assign) double stabilityImprovementPct;
@property (nonatomic, assign) double minZoom;
@property (nonatomic, assign) double maxZoom;
@property (nonatomic, assign) double avgZoom;
@property (nonatomic, assign) uint32_t width;
@property (nonatomic, assign) uint32_t height;
@property (nonatomic, assign) double nominalFps;
@property (nonatomic, copy) NSString *errorMessage;
@end

@interface GFStabilizerBridge : NSObject

@property (nonatomic, readonly) GFStabilizerPlanObjC *plan;

- (nullable instancetype)initWithSidecarPath:(NSString *)sidecarPath
                                  smoothness:(double)smoothness
                             velocityDamping:(double)damping
                                timeOffsetMs:(double)offsetMs
                                maxZoomLimit:(double)maxZoomLimit;

- (BOOL)warpPixelBuffer:(CVPixelBufferRef)srcPixelBuffer
        intoPixelBuffer:(CVPixelBufferRef)dstPixelBuffer
             frameIndex:(NSUInteger)frameIndex;

- (void)flushTextureCache;

@end

NS_ASSUME_NONNULL_END
