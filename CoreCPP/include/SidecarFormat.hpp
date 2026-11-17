#pragma once

#include <cstdint>

namespace gf {

#pragma pack(push, 1)

struct GFHeader {
    char magic[4];              // "GFS5"
    uint32_t version;           // 1
    double fps;                 // Nominal capture frame rate
    uint32_t width;             // Video frame width in pixels
    uint32_t height;            // Video frame height in pixels
    uint32_t mach_timebase_num; // mach_timebase_info.numer
    uint32_t mach_timebase_den; // mach_timebase_info.denom
    char uuid[37];              // Null-terminated UUID string (36 chars + '\0')
};

struct GFImuSample {
    double timestamp_sec;       // Mach boot-time seconds
    double gx;                  // Gyro X (rad/s)
    double gy;                  // Gyro Y (rad/s)
    double gz;                  // Gyro Z (rad/s)
    float ax;                   // Accel X (G)
    float ay;                   // Accel Y (G)
    float az;                   // Accel Z (G)
};

struct GFFrameSample {
    double pts_sec;             // Frame presentation timestamp in seconds
    float exposure_sec;         // Exposure duration in seconds
    float iso;                  // Sensor ISO sensitivity
    float readout_sec;          // Rolling shutter readout duration in seconds
};

#pragma pack(pop)

static_assert(sizeof(GFHeader) == 69, "GFHeader must be tightly packed to 69 bytes");
static_assert(sizeof(GFImuSample) == 44, "GFImuSample must be tightly packed to 44 bytes");
static_assert(sizeof(GFFrameSample) == 20, "GFFrameSample must be tightly packed to 20 bytes");

} // namespace gf
