#pragma once

#include <cstdint>

namespace gf {

#pragma pack(push, 1)

enum class GFRecordTag : uint8_t {
    Gyro = 0x01,
    Accel = 0x02,
    Frame = 0x03,
    Footer = 0xFF
};

struct GFFileHeader {
    char magic[4];              // "GFS5"
    uint32_t version;           // 2
    uint32_t capture_mode;      // 0 = 1080p30, 1 = 720p120
    uint32_t width;             // Video frame width in pixels
    uint32_t height;            // Video frame height in pixels
    double nominal_fps;         // Nominal capture frame rate
    double readout_time_sec;    // Rolling shutter readout duration (seconds)
    double gyro_bias_x;         // Calibrated gyro X bias (rad/s, iOS sensor frame)
    double gyro_bias_y;         // Calibrated gyro Y bias (rad/s, iOS sensor frame)
    double gyro_bias_z;         // Calibrated gyro Z bias (rad/s, iOS sensor frame)
    double lens_fx;             // Focal length X (pixels)
    double lens_fy;             // Focal length Y (pixels)
    double lens_cx;             // Principal point X (pixels)
    double lens_cy;             // Principal point Y (pixels)
    double dist_k1;             // Radial distortion k1
    double dist_k2;             // Radial distortion k2
    double dist_p1;             // Tangential distortion p1
    double dist_p2;             // Tangential distortion p2
    char uuid[37];              // Null-terminated UUID string matching <UUID>.mov
    uint8_t reserved[7];        // Reserved padding to 168 bytes
};

struct GFGyroRecord {
    double timestamp_sec;       // Mach boot-time seconds
    double gx;                  // Raw gyro X (rad/s, iOS sensor frame)
    double gy;                  // Raw gyro Y (rad/s, iOS sensor frame)
    double gz;                  // Raw gyro Z (rad/s, iOS sensor frame)
};

struct GFAccelRecord {
    double timestamp_sec;       // Mach boot-time seconds
    float ax;                   // Raw accel X (G, iOS sensor frame)
    float ay;                   // Raw accel Y (G, iOS sensor frame)
    float az;                   // Raw accel Z (G, iOS sensor frame)
};

struct GFFrameRecord {
    uint32_t frame_index;           // 0-based frame counter
    double pts_sec;                 // Presentation timestamp (seconds)
    float exposure_duration_sec;    // Frame exposure duration (seconds)
    float iso;                      // Sensor ISO sensitivity
    float lens_position;            // Focus lens position [0.0, 1.0]
    float rolling_shutter_sec;      // Readout time for this frame (seconds)
};

struct GFFooterRecord {
    uint64_t total_gyro_samples;
    uint64_t total_accel_samples;
    uint64_t total_frames;
    double recording_duration_sec;
};

#pragma pack(pop)

static_assert(sizeof(GFFileHeader) == 168, "GFFileHeader must be tightly packed to 168 bytes");
static_assert(sizeof(GFGyroRecord) == 32, "GFGyroRecord must be tightly packed to 32 bytes");
static_assert(sizeof(GFAccelRecord) == 20, "GFAccelRecord must be tightly packed to 20 bytes");
static_assert(sizeof(GFFrameRecord) == 28, "GFFrameRecord must be tightly packed to 28 bytes");
static_assert(sizeof(GFFooterRecord) == 32, "GFFooterRecord must be tightly packed to 32 bytes");

} // namespace gf
