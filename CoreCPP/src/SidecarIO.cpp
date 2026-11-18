#include "SidecarIO.hpp"

#include <algorithm>
#include <cstring>
#include <limits>

namespace gf {

SidecarWriter::~SidecarWriter() {
    closeWithoutFooter();
}

bool SidecarWriter::open(const std::string& path, const GFFileHeader& header) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_ != nullptr) {
        std::fclose(file_);
        file_ = nullptr;
    }

    file_ = std::fopen(path.c_str(), "wb");
    if (!file_) {
        return false;
    }

    std::setvbuf(file_, ioBuffer_, _IOFBF, sizeof(ioBuffer_));
    gyroCount_ = 0;
    accelCount_ = 0;
    frameCount_ = 0;

    GFFileHeader hdr = header;
    std::memcpy(hdr.magic, "GFS5", 4);
    hdr.version = 2;

    if (std::fwrite(&hdr, sizeof(GFFileHeader), 1, file_) != 1) {
        std::fclose(file_);
        file_ = nullptr;
        return false;
    }
    std::fflush(file_);
    return true;
}

bool SidecarWriter::writeTaggedRecord(GFRecordTag tag, const void* data, size_t size) {
    if (!file_) {
        return false;
    }
    const uint8_t tagByte = static_cast<uint8_t>(tag);
    if (std::fwrite(&tagByte, sizeof(uint8_t), 1, file_) != 1) {
        return false;
    }
    if (std::fwrite(data, size, 1, file_) != 1) {
        return false;
    }
    return true;
}

bool SidecarWriter::writeGyro(const GFGyroRecord& rec) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!writeTaggedRecord(GFRecordTag::Gyro, &rec, sizeof(GFGyroRecord))) {
        return false;
    }
    ++gyroCount_;
    return true;
}

bool SidecarWriter::writeAccel(const GFAccelRecord& rec) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!writeTaggedRecord(GFRecordTag::Accel, &rec, sizeof(GFAccelRecord))) {
        return false;
    }
    ++accelCount_;
    return true;
}

bool SidecarWriter::writeFrame(const GFFrameRecord& rec) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!writeTaggedRecord(GFRecordTag::Frame, &rec, sizeof(GFFrameRecord))) {
        return false;
    }
    ++frameCount_;
    return true;
}

bool SidecarWriter::closeWithFooter(double recordingDurationSec) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!file_) {
        return false;
    }
    GFFooterRecord footer{};
    footer.total_gyro_samples = gyroCount_;
    footer.total_accel_samples = accelCount_;
    footer.total_frames = frameCount_;
    footer.recording_duration_sec = recordingDurationSec;

    const bool ok = writeTaggedRecord(GFRecordTag::Footer, &footer, sizeof(GFFooterRecord));
    std::fflush(file_);
    std::fclose(file_);
    file_ = nullptr;
    return ok;
}

void SidecarWriter::closeWithoutFooter() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_) {
        std::fflush(file_);
        std::fclose(file_);
        file_ = nullptr;
    }
}

bool SidecarWriter::isOpen() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return file_ != nullptr;
}

SidecarSummary SidecarReader::inspectFile(const std::string& path) {
    GFFileHeader hdr{};
    std::vector<GFGyroRecord> gyro;
    std::vector<GFAccelRecord> accel;
    std::vector<GFFrameRecord> frames;
    SidecarSummary summary;
    readAll(path, hdr, gyro, accel, frames, summary);
    return summary;
}

bool SidecarReader::readAll(
    const std::string& path,
    GFFileHeader& outHeader,
    std::vector<GFGyroRecord>& outGyro,
    std::vector<GFAccelRecord>& outAccel,
    std::vector<GFFrameRecord>& outFrames,
    SidecarSummary& outSummary
) {
    outGyro.clear();
    outAccel.clear();
    outFrames.clear();
    outSummary = SidecarSummary{};

    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) {
        outSummary.errorMessage = "Failed to open sidecar file.";
        return false;
    }

    if (std::fread(&outHeader, sizeof(GFFileHeader), 1, f) != 1) {
        std::fclose(f);
        outSummary.errorMessage = "Truncated or missing GFFileHeader.";
        return false;
    }

    if (std::memcmp(outHeader.magic, "GFS5", 4) != 0 || outHeader.version != 2) {
        std::fclose(f);
        outSummary.errorMessage = "Invalid magic or unsupported sidecar version.";
        return false;
    }

    outSummary.validHeader = true;
    outSummary.header = outHeader;

    double firstAccelSec = 0.0;
    double lastAccelSec = 0.0;
    double minExpSec = std::numeric_limits<double>::max();
    double maxExpSec = 0.0;
    double prevPts = 0.0;
    bool hasPrevPts = false;
    const double dropThreshold = (outHeader.nominal_fps > 0.0) ? (1.5 / outHeader.nominal_fps) : 0.05;

    uint8_t tagByte = 0;
    while (std::fread(&tagByte, sizeof(uint8_t), 1, f) == 1) {
        const GFRecordTag tag = static_cast<GFRecordTag>(tagByte);
        if (tag == GFRecordTag::Gyro) {
            GFGyroRecord rec{};
            if (std::fread(&rec, sizeof(GFGyroRecord), 1, f) != 1) {
                break; // Truncated tail on crash
            }
            if (outGyro.empty()) {
                outSummary.firstGyroSec = rec.timestamp_sec;
            }
            outSummary.lastGyroSec = rec.timestamp_sec;
            outGyro.push_back(rec);
        } else if (tag == GFRecordTag::Accel) {
            GFAccelRecord rec{};
            if (std::fread(&rec, sizeof(GFAccelRecord), 1, f) != 1) {
                break;
            }
            if (outAccel.empty()) {
                firstAccelSec = rec.timestamp_sec;
            }
            lastAccelSec = rec.timestamp_sec;
            outAccel.push_back(rec);
        } else if (tag == GFRecordTag::Frame) {
            GFFrameRecord rec{};
            if (std::fread(&rec, sizeof(GFFrameRecord), 1, f) != 1) {
                break;
            }
            if (outFrames.empty()) {
                outSummary.firstPtsSec = rec.pts_sec;
            }
            outSummary.lastPtsSec = rec.pts_sec;

            if (hasPrevPts && (rec.pts_sec - prevPts) > dropThreshold) {
                ++outSummary.droppedFrames;
            }
            prevPts = rec.pts_sec;
            hasPrevPts = true;

            if (rec.exposure_duration_sec > 0.0f) {
                const double expSec = static_cast<double>(rec.exposure_duration_sec);
                minExpSec = std::min(minExpSec, expSec);
                maxExpSec = std::max(maxExpSec, expSec);
            }
            outFrames.push_back(rec);
        } else if (tag == GFRecordTag::Footer) {
            GFFooterRecord footer{};
            if (std::fread(&footer, sizeof(GFFooterRecord), 1, f) != 1) {
                break;
            }
            outSummary.hasFooter = true;
            outSummary.recordingDurationSec = footer.recording_duration_sec;
            break;
        } else {
            outSummary.errorMessage = "Unknown record tag encountered.";
            break;
        }
    }

    std::fclose(f);

    outSummary.gyroCount = outGyro.size();
    outSummary.accelCount = outAccel.size();
    outSummary.frameCount = outFrames.size();

    if (outGyro.size() >= 2 && outSummary.lastGyroSec > outSummary.firstGyroSec) {
        outSummary.gyroHz = static_cast<double>(outGyro.size() - 1) /
                            (outSummary.lastGyroSec - outSummary.firstGyroSec);
    }
    if (outAccel.size() >= 2 && lastAccelSec > firstAccelSec) {
        outSummary.accelHz = static_cast<double>(outAccel.size() - 1) /
                             (lastAccelSec - firstAccelSec);
    }
    if (outFrames.size() >= 2 && outSummary.lastPtsSec > outSummary.firstPtsSec) {
        outSummary.avgFps = static_cast<double>(outFrames.size() - 1) /
                            (outSummary.lastPtsSec - outSummary.firstPtsSec);
        if (outSummary.recordingDurationSec <= 0.0) {
            outSummary.recordingDurationSec = outSummary.lastPtsSec - outSummary.firstPtsSec;
        }
    }

    if (!outFrames.empty() && minExpSec <= maxExpSec) {
        outSummary.minExposureMs = minExpSec * 1000.0;
        outSummary.maxExposureMs = maxExpSec * 1000.0;
    }

    outSummary.syncCoverageOk =
        !outGyro.empty() &&
        !outFrames.empty() &&
        (outSummary.firstGyroSec <= outSummary.firstPtsSec + 0.05) &&
        (outSummary.lastGyroSec >= outSummary.lastPtsSec - 0.05);

    return true;
}

} // namespace gf
