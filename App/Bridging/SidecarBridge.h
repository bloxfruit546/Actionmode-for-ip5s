#import <AVFoundation/AVFoundation.h>
#import <CoreMedia/CoreMedia.h>
#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

typedef struct {
    double ptsSec;
    float exposureDurationSec;
    float iso;
    float lensPosition;
} GFFrameMetadataObjC;

@interface GFSidecarSummaryObjC : NSObject
@property (nonatomic, assign) BOOL validHeader;
@property (nonatomic, assign) BOOL hasFooter;
@property (nonatomic, assign) BOOL isVirtualGyro;
@property (nonatomic, assign) uint32_t captureMode;
@property (nonatomic, assign) uint32_t width;
@property (nonatomic, assign) uint32_t height;
@property (nonatomic, assign) double nominalFps;
@property (nonatomic, assign) double readoutTimeSec;
@property (nonatomic, assign) double gyroBiasX;
@property (nonatomic, assign) double gyroBiasY;
@property (nonatomic, assign) double gyroBiasZ;
@property (nonatomic, assign) uint64_t gyroCount;
@property (nonatomic, assign) uint64_t accelCount;
@property (nonatomic, assign) uint64_t frameCount;
@property (nonatomic, assign) double gyroHz;
@property (nonatomic, assign) double accelHz;
@property (nonatomic, assign) double avgFps;
@property (nonatomic, assign) uint32_t droppedFrames;
@property (nonatomic, assign) BOOL syncCoverageOk;
@property (nonatomic, assign) double minExposureMs;
@property (nonatomic, assign) double maxExposureMs;
@property (nonatomic, assign) double recordingDurationSec;
@property (nonatomic, copy) NSString *uuid;
@property (nonatomic, copy) NSString *errorMessage;
@end

@interface GFSidecarWriterBridge : NSObject

- (BOOL)openAtPath:(NSString *)path
              uuid:(NSString *)uuid
       captureMode:(uint32_t)mode
             width:(uint32_t)width
            height:(uint32_t)height
        nominalFps:(double)fps
    readoutTimeSec:(double)readoutSec
         gyroBiasX:(double)bx
         gyroBiasY:(double)by
         gyroBiasZ:(double)bz
                fx:(double)fx
                fy:(double)fy
                cx:(double)cx
                cy:(double)cy
                k1:(double)k1
                k2:(double)k2
                p1:(double)p1
                p2:(double)p2;

- (BOOL)writeGyroTimestamp:(double)ts gx:(double)gx gy:(double)gy gz:(double)gz;
- (BOOL)writeAccelTimestamp:(double)ts ax:(float)ax ay:(float)ay az:(float)az;
- (BOOL)writeFrameIndex:(uint32_t)index
                    pts:(double)pts
            exposureSec:(float)expSec
                    iso:(float)iso
           lensPosition:(float)lensPos
      rollingShutterSec:(float)rsSec;

- (BOOL)closeWithDuration:(double)durationSec;
- (void)closeWithoutFooter;

@end

@interface GFSidecarReaderBridge : NSObject
+ (GFSidecarSummaryObjC *)inspectFileAtPath:(NSString *)path;
+ (GFFrameMetadataObjC)extractFrameMetadata:(CMSampleBufferRef)sampleBuffer
                              fallbackDevice:(nullable AVCaptureDevice *)device;
@end

@interface GFGyroBiasCalibratorResultObjC : NSObject
@property (nonatomic, assign) NSInteger status;
@property (nonatomic, assign) double biasX;
@property (nonatomic, assign) double biasY;
@property (nonatomic, assign) double biasZ;
@property (nonatomic, assign) double gravityX;
@property (nonatomic, assign) double gravityY;
@property (nonatomic, assign) double gravityZ;
@property (nonatomic, assign) double maxGyroStdDev;
@property (nonatomic, assign) uint64_t sampleCount;
@end

@interface GFGyroBiasCalibratorBridge : NSObject
- (void)reset;
- (void)addSampleGx:(double)gx gy:(double)gy gz:(double)gz ax:(double)ax ay:(double)ay az:(double)az;
- (GFGyroBiasCalibratorResultObjC *)finalizeWithMaxStdDev:(double)maxStdDev;
@end

@interface GFVirtualGyroResultObjC : NSObject
@property (nonatomic, assign) BOOL success;
@property (nonatomic, assign) uint64_t frameCount;
@property (nonatomic, assign) uint64_t gyroCount;
@property (nonatomic, assign) double meanInlierRatio;
@property (nonatomic, copy) NSString *errorMessage;
@end

@interface GFVirtualGyroBuilderBridge : NSObject
- (instancetype)initWithWidth:(uint32_t)width
                       height:(uint32_t)height
                   nominalFps:(double)fps
               readoutTimeSec:(double)readoutSec
                           fx:(double)fx
                           fy:(double)fy
                           cx:(double)cx
                           cy:(double)cy
                         uuid:(NSString *)uuid;
- (BOOL)pushLumaData:(NSData *)lumaData width:(int)width height:(int)height timestampSec:(double)ts;
- (GFVirtualGyroResultObjC *)finalizeAndWriteToPath:(NSString *)outputSidecarPath;
@end

NS_ASSUME_NONNULL_END
