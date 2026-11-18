#include "CalibrationProfile.hpp"
#include "SidecarIO.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

bool approxEqual(double a, double b, double tol = 1e-6) {
    return std::fabs(a - b) <= tol;
}

void testSidecarStreamingWriteAndRead() {
    const std::string path = "test_stream_v2.gfraw";
    std::remove(path.c_str());

    gf::GFFileHeader hdr{};
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;
    hdr.capture_mode = 0;
    hdr.width = 1920;
    hdr.height = 1080;
    hdr.nominal_fps = 30.0;
    hdr.readout_time_sec = 0.0152;
    hdr.gyro_bias_x = 0.001;
    hdr.gyro_bias_y = -0.002;
    hdr.gyro_bias_z = 0.0005;
    hdr.lens_fx = 1660.0;
    hdr.lens_fy = 1660.0;
    hdr.lens_cx = 960.0;
    hdr.lens_cy = 540.0;
    hdr.dist_k1 = 0.042;
    hdr.dist_k2 = -0.115;
    std::snprintf(hdr.uuid, sizeof(hdr.uuid), "12345678-1234-4123-8123-1234567890AB");

    {
        gf::SidecarWriter writer;
        assert(writer.open(path, hdr));

        // 10,000 IMU samples (~111s at 90Hz)
        for (int i = 0; i < 10000; ++i) {
            const double t = 1000.0 + i * (1.0 / 90.0);
            assert(writer.writeGyro({t, 0.01 * i, -0.02, 0.005}));
            assert(writer.writeAccel({t, 0.0f, -1.0f, 0.02f}));
        }

        // 1,800 frames (60s at 30fps) with 1 simulated dropped frame at index 500
        for (uint32_t f = 0; f < 1800; ++f) {
            const double extraGap = (f >= 500) ? (1.0 / 30.0) : 0.0;
            const double pts = 1000.05 + f * (1.0 / 30.0) + extraGap;
            assert(writer.writeFrame({f, pts, 0.0083f, 100.0f, 0.75f, 0.0152f}));
        }

        assert(writer.closeWithFooter(60.0));
    }

    gf::GFFileHeader readHdr{};
    std::vector<gf::GFGyroRecord> gyro;
    std::vector<gf::GFAccelRecord> accel;
    std::vector<gf::GFFrameRecord> frames;
    gf::SidecarSummary summary;

    assert(gf::SidecarReader::readAll(path, readHdr, gyro, accel, frames, summary));
    assert(summary.validHeader);
    assert(summary.hasFooter);
    assert(summary.gyroCount == 10000);
    assert(summary.accelCount == 10000);
    assert(summary.frameCount == 1800);
    assert(approxEqual(summary.gyroHz, 90.0, 0.1));
    assert(approxEqual(summary.accelHz, 90.0, 0.1));
    assert(summary.droppedFrames == 1);
    assert(summary.syncCoverageOk);
    assert(approxEqual(summary.minExposureMs, 8.3, 0.01));
    assert(std::strcmp(readHdr.uuid, hdr.uuid) == 0);

    std::remove(path.c_str());
}

void testSidecarCrashRecoveryWithoutFooter() {
    const std::string path = "test_crash_v2.gfraw";
    std::remove(path.c_str());

    gf::GFFileHeader hdr{};
    hdr.width = 1280;
    hdr.height = 720;
    hdr.nominal_fps = 120.0;

    {
        gf::SidecarWriter writer;
        assert(writer.open(path, hdr));
        for (int i = 0; i < 120; ++i) {
            assert(writer.writeGyro({10.0 + i * 0.01, 0.0, 0.0, 0.0}));
        }
        for (uint32_t f = 0; f < 30; ++f) {
            assert(writer.writeFrame({f, 10.01 + f * (1.0 / 120.0), 0.004f, 200.0f, 0.5f, 0.0074f}));
        }
        writer.closeWithoutFooter();
    }

    const gf::SidecarSummary summary = gf::SidecarReader::inspectFile(path);
    assert(summary.validHeader);
    assert(!summary.hasFooter);
    assert(summary.gyroCount == 120);
    assert(summary.frameCount == 30);
    assert(summary.syncCoverageOk);

    std::remove(path.c_str());
}

void testAxisMappingAndCalibrator() {
    // Verify right-handedness (det = +1) of mapIOSImuToCameraFrame:
    const gf::Vec3 cx = gf::mapIOSImuToCameraFrame({1.0, 0.0, 0.0});
    const gf::Vec3 cy = gf::mapIOSImuToCameraFrame({0.0, 1.0, 0.0});
    const gf::Vec3 cz = gf::mapIOSImuToCameraFrame({0.0, 0.0, 1.0});
    const double det = cx.dot(cy.cross(cz));
    assert(approxEqual(det, 1.0, 1e-12));

    // Static calibrator PASS case
    gf::StaticImuCalibrator cal;
    for (int i = 0; i < 200; ++i) {
        const double jitter = (i % 2 == 0) ? 0.001 : -0.001;
        cal.addSample(0.012 + jitter, -0.008 + jitter, 0.003, 0.0, -1.0, 0.0);
    }
    const auto okRes = cal.finalize(0.015);
    assert(okRes.status == gf::StaticImuCalibrator::Status::Ok);
    assert(approxEqual(okRes.biasX, 0.012, 1e-6));
    assert(approxEqual(okRes.biasY, -0.008, 1e-6));
    assert(approxEqual(okRes.biasZ, 0.003, 1e-6));
    assert(approxEqual(okRes.gravityY, -1.0, 1e-6));

    // Static calibrator REJECTED_MOTION_DETECTED case
    cal.reset();
    for (int i = 0; i < 200; ++i) {
        const double shake = (i % 2 == 0) ? 0.15 : -0.15;
        cal.addSample(shake, 0.0, 0.0, 0.0, -1.0, 0.0);
    }
    const auto failRes = cal.finalize(0.015);
    assert(failRes.status == gf::StaticImuCalibrator::Status::RejectedMotionDetected);
}

} // namespace

int main() {
    testSidecarStreamingWriteAndRead();
    testSidecarCrashRecoveryWithoutFooter();
    testAxisMappingAndCalibrator();
    std::printf("All CoreCPP SidecarIO and Calibration tests passed.\n");
    return 0;
}
