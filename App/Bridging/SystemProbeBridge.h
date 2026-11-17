#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@interface GFSystemProbe : NSObject

+ (uint64_t)currentPhysFootprintBytes;
+ (double)currentMachBootTimeSeconds;
+ (NSString *)verifyCppCoreSelfTest;
+ (NSDictionary<NSString *, id> *)probeMetalA7Capabilities;

@end

NS_ASSUME_NONNULL_END
