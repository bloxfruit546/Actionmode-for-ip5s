#include "CalibrationProfile.hpp"

#include <algorithm>
#include <cmath>

namespace gf {

CalibrationProfile CalibrationProfile::defaultIPhone5s1080p30() {
    CalibrationProfile p;
    p.width = 1920;
    p.height = 1080;
    p.fx = 1815.0;
    p.fy = 1815.0;
    p.cx = 960.0;
    p.cy = 540.0;
    p.k1 = 0.012;
    p.k2 = -0.022;
    p.p1 = 0.0;
    p.p2 = 0.0;
    p.readoutTimeSec = 0.0152;
    return p;
}

CalibrationProfile CalibrationProfile::defaultIPhone5s720p120() {
    CalibrationProfile p;
    p.width = 1280;
    p.height = 720;
    p.fx = 1210.0;
    p.fy = 1210.0;
    p.cx = 640.0;
    p.cy = 360.0;
    p.k1 = 0.012;
    p.k2 = -0.022;
    p.p1 = 0.0;
    p.p2 = 0.0;
    p.readoutTimeSec = 0.0074;
    return p;
}

Vec3 mapIOSImuToCameraFrame(const Vec3& iosVec) {
    return Vec3(-iosVec.y, -iosVec.x, -iosVec.z);
}

void StaticImuCalibrator::reset() {
    gyroSamples_.clear();
    accelSamples_.clear();
}

void StaticImuCalibrator::addSample(double gx, double gy, double gz, double ax, double ay, double az) {
    gyroSamples_.emplace_back(gx, gy, gz);
    accelSamples_.emplace_back(ax, ay, az);
}

StaticImuCalibrator::Result StaticImuCalibrator::finalize(double maxAllowedStdDevRadPerSec) const {
    Result res;
    res.sampleCount = static_cast<uint64_t>(gyroSamples_.size());
    if (gyroSamples_.size() < 30 || accelSamples_.empty()) {
        res.status = Status::InsufficientSamples;
        return res;
    }

    const double n = static_cast<double>(gyroSamples_.size());
    Vec3 meanG(0.0, 0.0, 0.0);
    for (const Vec3& g : gyroSamples_) {
        meanG = meanG + g;
    }
    meanG = meanG * (1.0 / n);

    Vec3 varG(0.0, 0.0, 0.0);
    for (const Vec3& g : gyroSamples_) {
        const Vec3 d = g - meanG;
        varG.x += d.x * d.x;
        varG.y += d.y * d.y;
        varG.z += d.z * d.z;
    }
    const double stdX = std::sqrt(varG.x / n);
    const double stdY = std::sqrt(varG.y / n);
    const double stdZ = std::sqrt(varG.z / n);
    res.maxGyroStdDev = std::max({stdX, stdY, stdZ});

    if (res.maxGyroStdDev > maxAllowedStdDevRadPerSec) {
        res.status = Status::RejectedMotionDetected;
        return res;
    }

    Vec3 meanA(0.0, 0.0, 0.0);
    for (const Vec3& a : accelSamples_) {
        meanA = meanA + a;
    }
    meanA = (meanA * (1.0 / static_cast<double>(accelSamples_.size()))).normalized();

    res.status = Status::Ok;
    res.biasX = meanG.x;
    res.biasY = meanG.y;
    res.biasZ = meanG.z;
    res.gravityX = meanA.x;
    res.gravityY = meanA.y;
    res.gravityZ = meanA.z;
    return res;
}

} // namespace gf
