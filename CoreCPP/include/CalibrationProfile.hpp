#pragma once

#include "Quaternion.hpp"

#include <cstdint>
#include <vector>

namespace gf {

struct CalibrationProfile {
    uint32_t width = 1920;
    uint32_t height = 1080;
    double fx = 1660.0;
    double fy = 1660.0;
    double cx = 960.0;
    double cy = 540.0;
    double k1 = 0.042;
    double k2 = -0.115;
    double p1 = 0.0;
    double p2 = 0.0;
    double readoutTimeSec = 0.0152;
    double gyroBiasX = 0.0;
    double gyroBiasY = 0.0;
    double gyroBiasZ = 0.0;
    double gravityX = 0.0;
    double gravityY = -1.0;
    double gravityZ = 0.0;

    static CalibrationProfile defaultIPhone5s1080p30();
    static CalibrationProfile defaultIPhone5s720p120();
};

/**
 * Maps raw iOS CoreMotion sensor vector (Portrait: +X right, +Y up, +Z out of screen)
 * to standard Camera/OpenCV frame for Landscape Left (Home button on right, rear camera):
 *   +X_cam (image right) = -Y_ios
 *   +Y_cam (image down)  = -X_ios
 *   +Z_cam (into scene)  = -Z_ios
 * Orthogonal right-handed rotation (det = +1).
 */
Vec3 mapIOSImuToCameraFrame(const Vec3& iosVec);

class StaticImuCalibrator {
public:
    enum class Status {
        Ok = 0,
        InsufficientSamples = 1,
        RejectedMotionDetected = 2
    };

    struct Result {
        Status status = Status::InsufficientSamples;
        double biasX = 0.0;
        double biasY = 0.0;
        double biasZ = 0.0;
        double gravityX = 0.0;
        double gravityY = 0.0;
        double gravityZ = 0.0;
        double maxGyroStdDev = 0.0;
        uint64_t sampleCount = 0;
    };

    void reset();
    void addSample(double gx, double gy, double gz, double ax, double ay, double az);
    Result finalize(double maxAllowedStdDevRadPerSec = 0.015) const;

private:
    std::vector<Vec3> gyroSamples_;
    std::vector<Vec3> accelSamples_;
};

} // namespace gf
