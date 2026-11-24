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
@property (nonatomic, assign) double rollGyroScale;
@property (nonatomic, assign) BOOL hasRollScaleEstimate;
@property (nonatomic, copy) NSString *diagnosticMessage;
@end

@interface GFOpticalSyncCollectorBridge : NSObject
- (void)reset;
- (void)addLumaPixelBuffer:(CVPixelBufferRef)pixelBuffer timestampSec:(double)ts;
- (GFOpticalSyncResultObjC *)solveWithSidecarPath:(NSString *)sidecarPath motoActionMode:(BOOL)motoActionMode;
@end

@interface GFRefinedCalibrationObjC : NSObject
@property (nonatomic, assign) BOOL valid;
@property (nonatomic, assign) BOOL usedPriorFallback;
@property (nonatomic, assign) NSInteger axisPresetIndex;
@property (nonatomic, assign) double offsetA;
@property (nonatomic, assign) double offsetB;
@property (nonatomic, assign) double scaleX;
@property (nonatomic, assign) double scaleY;
@property (nonatomic, assign) double scaleZ;
@property (nonatomic, assign) double biasX;
@property (nonatomic, assign) double biasY;
@property (nonatomic, assign) double biasZ;
@property (nonatomic, assign) double readoutMs;
@property (nonatomic, assign) double focalScale;
@property (nonatomic, assign) double confidence;
@property (nonatomic, assign) double residualRmsBefore;
@property (nonatomic, assign) double residualRmsAfter;
@property (nonatomic, copy) NSString *summaryString;
- (BOOL)saveToJSONPath:(NSString *)path;
+ (nullable GFRefinedCalibrationObjC *)loadFromJSONPath:(NSString *)path;
@end

@interface GFCalibrationRefinerBridge : NSObject
- (void)resetWithSidecarPath:(NSString *)sidecarPath
                    hasPrior:(BOOL)hasPrior
              priorReadoutMs:(double)priorReadoutMs
             priorFocalScale:(double)priorFocalScale
                 priorScaleX:(double)priorScaleX
                 priorScaleY:(double)priorScaleY
                 priorScaleZ:(double)priorScaleZ
              motoActionMode:(BOOL)motoActionMode;
- (void)pushLumaPixelBuffer:(CVPixelBufferRef)pixelBuffer timestampSec:(double)ts frameIndex:(NSUInteger)frameIndex;
- (GFRefinedCalibrationObjC *)finalizeCalibration;
+ (double)medianOfNumbers:(NSArray<NSNumber *> *)values;
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
@property (nonatomic, assign) double hybridResidualMaxPx;
@property (nonatomic, assign) NSUInteger translational2DValidFrames;
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
@property (nonatomic, assign) BOOL chromaSitingFixEnabled;
@property (nonatomic, assign) BOOL temporalSharpenEnabled;

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
                                 horizonMode:(NSInteger)horizonMode
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
                          sharpenStrength:(double)sharpenStrength
                    useRefinedCalibration:(BOOL)useRefinedCalibration
                         refinedCalibPath:(nullable NSString *)refinedCalibPath
                       useConstrainedPath:(BOOL)useConstrainedPath;

- (void)resetTranslational2DWithStrength:(double)strength;
- (void)feedTranslational2DFrameIndex:(NSUInteger)frameIndex pixelBuffer:(CVPixelBufferRef)pixelBuffer;
- (void)finalizeTranslational2D;

- (BOOL)warpPixelBuffer:(CVPixelBufferRef)srcPixelBuffer
        intoPixelBuffer:(CVPixelBufferRef)dstPixelBuffer
             frameIndex:(NSUInteger)frameIndex;

- (NSString *)computeQualityReportJSONWithClipName:(NSString *)clipName
                                    inputPtsSec:(nullable NSArray<NSNumber *> *)inputPtsSec
                                   outputPtsSec:(nullable NSArray<NSNumber *> *)outputPtsSec;

+ (NSString *)validatePtsSummaryWithInputPtsSec:(NSArray<NSNumber *> *)inputPtsSec
                                   outputPtsSec:(NSArray<NSNumber *> *)outputPtsSec
                                     nominalFps:(double)nominalFps
                                         passed:(nullable BOOL *)outPassed;

- (void)flushTextureCache;

@end

NS_ASSUME_NONNULL_END
