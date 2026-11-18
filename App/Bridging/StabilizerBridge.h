#import <CoreVideo/CoreVideo.h>
#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@interface GFOpticalSyncResultObjC : NSObject
@property (nonatomic, assign) BOOL valid;
@property (nonatomic, assign) double bestOffsetMs;
@property (nonatomic, assign) double bestNCC;
@property (nonatomic, assign) NSInteger bestAxisPresetIndex;
@property (nonatomic, copy) NSString *axisPresetName;
@property (nonatomic, assign) double signedReadoutTimeMs;
@property (nonatomic, assign) BOOL hasReadoutEstimate;
@property (nonatomic, assign) double effectiveFocalScale;
@property (nonatomic, assign) BOOL hasFocalScaleEstimate;
@property (nonatomic, copy) NSString *diagnosticMessage;
@end

@interface GFOpticalSyncCollectorBridge : NSObject
- (void)reset;
- (void)addLumaPixelBuffer:(CVPixelBufferRef)pixelBuffer timestampSec:(double)ts;
- (GFOpticalSyncResultObjC *)solveWithSidecarPath:(NSString *)sidecarPath;
@end

@interface GFStabilizerPlanObjC : NSObject
@property (nonatomic, assign) BOOL valid;
@property (nonatomic, assign) NSUInteger frameCount;
@property (nonatomic, assign) double stabilityImprovementPct;
@property (nonatomic, assign) double minZoom;
@property (nonatomic, assign) double maxZoom;
@property (nonatomic, assign) double avgZoom;
@property (nonatomic, assign) double maxInputRollDeg;
@property (nonatomic, assign) double maxOutputRollErrorDeg;
@property (nonatomic, assign) double maxCorrectionDeg;
@property (nonatomic, copy) NSString *detectedOrientationName;
@property (nonatomic, copy) NSString *gpuMatrixProof;
@property (nonatomic, assign) BOOL isIdentityMatrix;
@property (nonatomic, assign) uint32_t width;
@property (nonatomic, assign) uint32_t height;
@property (nonatomic, assign) double nominalFps;
@property (nonatomic, copy) NSString *errorMessage;
@end

@interface GFStabilizerBridge : NSObject

@property (nonatomic, readonly) GFStabilizerPlanObjC *plan;

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
                       focalScaleOverride:(double)focalScaleOverride;

- (BOOL)warpPixelBuffer:(CVPixelBufferRef)srcPixelBuffer
        intoPixelBuffer:(CVPixelBufferRef)dstPixelBuffer
             frameIndex:(NSUInteger)frameIndex;

- (void)flushTextureCache;

@end

NS_ASSUME_NONNULL_END
