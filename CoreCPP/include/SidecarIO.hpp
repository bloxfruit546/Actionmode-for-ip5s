#pragma once

#include "SidecarFormat.hpp"

#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

namespace gf {

struct SidecarSummary {
    bool validHeader = false;
    bool hasFooter = false;
    GFFileHeader header{};
    uint64_t gyroCount = 0;
    uint64_t accelCount = 0;
    uint64_t frameCount = 0;
    double gyroHz = 0.0;
    double accelHz = 0.0;
    double avgFps = 0.0;
    uint32_t droppedFrames = 0;
    bool syncCoverageOk = false;
    double minExposureMs = 0.0;
    double maxExposureMs = 0.0;
    double firstPtsSec = 0.0;
    double lastPtsSec = 0.0;
    double firstGyroSec = 0.0;
    double lastGyroSec = 0.0;
    double recordingDurationSec = 0.0;
    std::string errorMessage;
};

class SidecarWriter {
public:
    SidecarWriter() = default;
    ~SidecarWriter();

    SidecarWriter(const SidecarWriter&) = delete;
    SidecarWriter& operator=(const SidecarWriter&) = delete;

    bool open(const std::string& path, const GFFileHeader& header);
    bool writeGyro(const GFGyroRecord& rec);
    bool writeAccel(const GFAccelRecord& rec);
    bool writeFrame(const GFFrameRecord& rec);
    bool closeWithFooter(double recordingDurationSec);
    void closeWithoutFooter();

    bool isOpen() const;

private:
    bool writeTaggedRecord(GFRecordTag tag, const void* data, size_t size);

    mutable std::mutex mutex_;
    std::FILE* file_ = nullptr;
    char ioBuffer_[65536]{};
    uint64_t gyroCount_ = 0;
    uint64_t accelCount_ = 0;
    uint64_t frameCount_ = 0;
};

class SidecarReader {
public:
    static SidecarSummary inspectFile(const std::string& path);
    static bool readAll(
        const std::string& path,
        GFFileHeader& outHeader,
        std::vector<GFGyroRecord>& outGyro,
        std::vector<GFAccelRecord>& outAccel,
        std::vector<GFFrameRecord>& outFrames,
        SidecarSummary& outSummary
    );
};

} // namespace gf
