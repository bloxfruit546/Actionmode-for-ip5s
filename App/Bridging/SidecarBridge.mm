#import "SidecarBridge.h"
#import "CalibrationProfile.hpp"
#import "SidecarIO.hpp"
#import "VirtualGyroEngine.hpp"

#import <ImageIO/ImageIO.h>
#include <cstring>
#include <memory>

@implementation GFSidecarSummaryObjC
- (instancetype)init {
    self = [super init];
    if (self) {
        _uuid = @"";
        _errorMessage = @"";
    }
    return self;
}
@end

@implementation GFSidecarWriterBridge {
    std::unique_ptr<gf::SidecarWriter> _writer;
}

- (instancetype)init {
    self = [super init];
    if (self) {
        _writer = std::make_unique<gf::SidecarWriter>();
    }
    return self;
}

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
                p2:(double)p2 {
    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.capture_mode = mode;
    hdr.width = width;
    hdr.height = height;
    hdr.nominal_fps = fps;
    hdr.readout_time_sec = readoutSec;
    hdr.gyro_bias_x = bx;
    hdr.gyro_bias_y = by;
    hdr.gyro_bias_z = bz;
    hdr.lens_fx = fx;
    hdr.lens_fy = fy;
    hdr.lens_cx = cx;
    hdr.lens_cy = cy;
    hdr.dist_k1 = k1;
    hdr.dist_k2 = k2;
    hdr.dist_p1 = p1;
    hdr.dist_p2 = p2;

    const char *uuidC = [uuid UTF8String];
    if (uuidC) {
        std::strncpy(hdr.uuid, uuidC, sizeof(hdr.uuid) - 1);
        hdr.uuid[sizeof(hdr.uuid) - 1] = '\0';
    }

    return _writer->open([path UTF8String], hdr) ? YES : NO;
}

- (BOOL)writeGyroTimestamp:(double)ts gx:(double)gx gy:(double)gy gz:(double)gz {
    return _writer->writeGyro({ts, gx, gy, gz}) ? YES : NO;
}

- (BOOL)writeAccelTimestamp:(double)ts ax:(float)ax ay:(float)ay az:(float)az {
    return _writer->writeAccel({ts, ax, ay, az}) ? YES : NO;
}

- (BOOL)writeFrameIndex:(uint32_t)index
                    pts:(double)pts
            exposureSec:(float)expSec
                    iso:(float)iso
           lensPosition:(float)lensPos
      rollingShutterSec:(float)rsSec {
    return _writer->writeFrame({index, pts, expSec, iso, lensPos, rsSec}) ? YES : NO;
}

- (BOOL)closeWithDuration:(double)durationSec {
    return _writer->closeWithFooter(durationSec) ? YES : NO;
}

- (void)closeWithoutFooter {
    _writer->closeWithoutFooter();
}

@end

@implementation GFSidecarReaderBridge

+ (GFSidecarSummaryObjC *)inspectFileAtPath:(NSString *)path {
    const gf::SidecarSummary s = gf::SidecarReader::inspectFile([path UTF8String]);
    GFSidecarSummaryObjC *out = [[GFSidecarSummaryObjC alloc] init];
    out.validHeader = s.validHeader ? YES : NO;
    out.hasFooter = s.hasFooter ? YES : NO;
    out.isVirtualGyro = (s.header.reserved[0] != 0) ? YES : NO;
    out.captureMode = s.header.capture_mode;
    out.width = s.header.width;
    out.height = s.header.height;
    out.nominalFps = s.header.nominal_fps;
    out.readoutTimeSec = s.header.readout_time_sec;
    out.gyroBiasX = s.header.gyro_bias_x;
    out.gyroBiasY = s.header.gyro_bias_y;
    out.gyroBiasZ = s.header.gyro_bias_z;
    out.gyroCount = s.gyroCount;
    out.accelCount = s.accelCount;
    out.frameCount = s.frameCount;
    out.gyroHz = s.gyroHz;
    out.accelHz = s.accelHz;
    out.avgFps = s.avgFps;
    out.droppedFrames = s.droppedFrames;
    out.syncCoverageOk = s.syncCoverageOk ? YES : NO;
    out.minExposureMs = s.minExposureMs;
    out.maxExposureMs = s.maxExposureMs;
    out.recordingDurationSec = s.recordingDurationSec;
    out.uuid = [NSString stringWithUTF8String:s.header.uuid] ?: @"";
    out.errorMessage = [NSString stringWithUTF8String:s.errorMessage.c_str()] ?: @"";
    return out;
}

+ (GFFrameMetadataObjC)extractFrameMetadata:(CMSampleBufferRef)sampleBuffer
                              fallbackDevice:(nullable AVCaptureDevice *)device {
    GFFrameMetadataObjC meta{};
    meta.ptsSec = CMTimeGetSeconds(CMSampleBufferGetPresentationTimeStamp(sampleBuffer));

    float expSec = 0.0f;
    float iso = 0.0f;
    float lensPos = 0.0f;

    CFDictionaryRef exifDict = (CFDictionaryRef)CMGetAttachment(
        sampleBuffer,
        kCGImagePropertyExifDictionary,
        nullptr
    );
    if (exifDict) {
        NSDictionary *exif = (__bridge NSDictionary *)exifDict;
        NSNumber *expNum = exif[(NSString *)kCGImagePropertyExifExposureTime];
        if (expNum) {
            expSec = [expNum floatValue];
        }
        NSArray *isoArr = exif[(NSString *)kCGImagePropertyExifISOSpeedRatings];
        if ([isoArr isKindOfClass:[NSArray class]] && isoArr.count > 0) {
            iso = [isoArr.firstObject floatValue];
        }
    }

    if (device) {
        if (expSec <= 0.0f) {
            expSec = (float)CMTimeGetSeconds(device.exposureDuration);
        }
        if (iso <= 0.0f) {
            iso = device.ISO;
        }
        lensPos = device.lensPosition;
    }

    meta.exposureDurationSec = expSec;
    meta.iso = iso;
    meta.lensPosition = lensPos;
    return meta;
}

@end

@implementation GFGyroBiasCalibratorResultObjC
@end

@implementation GFGyroBiasCalibratorBridge {
    std::unique_ptr<gf::StaticImuCalibrator> _calibrator;
}

- (instancetype)init {
    self = [super init];
    if (self) {
        _calibrator = std::make_unique<gf::StaticImuCalibrator>();
    }
    return self;
}

- (void)reset {
    _calibrator->reset();
}

- (void)addSampleGx:(double)gx gy:(double)gy gz:(double)gz ax:(double)ax ay:(double)ay az:(double)az {
    _calibrator->addSample(gx, gy, gz, ax, ay, az);
}

- (GFGyroBiasCalibratorResultObjC *)finalizeWithMaxStdDev:(double)maxStdDev {
    const auto res = _calibrator->finalize(maxStdDev);
    GFGyroBiasCalibratorResultObjC *out = [[GFGyroBiasCalibratorResultObjC alloc] init];
    out.status = static_cast<NSInteger>(res.status);
    out.biasX = res.biasX;
    out.biasY = res.biasY;
    out.biasZ = res.biasZ;
    out.gravityX = res.gravityX;
    out.gravityY = res.gravityY;
    out.gravityZ = res.gravityZ;
    out.maxGyroStdDev = res.maxGyroStdDev;
    out.sampleCount = res.sampleCount;
    return out;
}

@end

@implementation GFVirtualGyroResultObjC
- (instancetype)init {
    self = [super init];
    if (self) {
        _errorMessage = @"";
    }
    return self;
}
@end

@implementation GFVirtualGyroBuilderBridge {
    std::unique_ptr<gf::VirtualGyroEngine> _engine;
}

- (instancetype)initWithWidth:(uint32_t)width
                       height:(uint32_t)height
                   nominalFps:(double)fps
               readoutTimeSec:(double)readoutSec
                           fx:(double)fx
                           fy:(double)fy
                           cx:(double)cx
                           cy:(double)cy
                         uuid:(NSString *)uuid {
    self = [super init];
    if (self) {
        gf::VirtualGyroConfig cfg;
        cfg.videoWidth = width;
        cfg.videoHeight = height;
        cfg.nominalFps = fps;
        cfg.readoutTimeSec = readoutSec;
        cfg.lensFx = fx;
        cfg.lensFy = fy;
        cfg.lensCx = cx;
        cfg.lensCy = cy;
        cfg.uuid = uuid ? std::string([uuid UTF8String]) : "";
        _engine = std::make_unique<gf::VirtualGyroEngine>(cfg);
    }
    return self;
}

- (BOOL)pushLumaData:(NSData *)lumaData width:(int)width height:(int)height timestampSec:(double)ts {
    if (!lumaData || lumaData.length < (NSUInteger)(width * height)) {
        return NO;
    }
    return _engine->pushLumaFrame((const uint8_t *)lumaData.bytes, width, height, ts) ? YES : NO;
}

- (GFVirtualGyroResultObjC *)finalizeAndWriteToPath:(NSString *)outputSidecarPath {
    const gf::VirtualGyroResult res = _engine->finalizeAndWrite([outputSidecarPath UTF8String]);
    GFVirtualGyroResultObjC *out = [[GFVirtualGyroResultObjC alloc] init];
    out.success = res.success ? YES : NO;
    out.frameCount = res.frameCount;
    out.gyroCount = res.gyroCount;
    out.meanInlierRatio = res.meanInlierRatio;
    out.errorMessage = [NSString stringWithUTF8String:res.errorMessage.c_str()] ?: @"";
    return out;
}

@end
