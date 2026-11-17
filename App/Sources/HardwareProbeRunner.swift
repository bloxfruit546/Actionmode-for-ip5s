import AVFoundation
import CoreMotion
import CoreVideo
import Foundation
import Photos
import UIKit

final class HardwareProbeRunner: NSObject {
    private let workQueue = DispatchQueue(label: "com.gyrostabilizer5s.probe", qos: .userInitiated)
    private var isRunning = false

    func runAllProbes(completion: @escaping () -> Void) {
        workQueue.async { [weak self] in
            guard let self = self else { return }
            if self.isRunning {
                FileLogger.shared.log("Probe already running; ignoring duplicate request.")
                return
            }
            self.isRunning = true

            FileLogger.shared.log("=== STARTING FULL iPHONE 5s (A7 / iOS 12) HARDWARE PROBE ===")
            self.probe1CppAndMemoryBaseline()
            self.probe2MetalA7Capabilities()
            self.probe3CoreMotionFrequencyAndClock()
            self.probe4Camera1080p30AndPTS()
            self.probe5H264MaxBitrate()
            self.probe6PhotosAndSafeRamCeiling()
            FileLogger.shared.log("=== ALL 6 HARDWARE PROBES COMPLETED ===")

            self.isRunning = false
            DispatchQueue.main.async {
                completion()
            }
        }
    }

    // MARK: - Probe 1: C++17 Core & Memory Baseline
    private func probe1CppAndMemoryBaseline() {
        FileLogger.shared.log("[1/6] C++17 Core & Memory Baseline...")
        let initialMB = Double(GFSystemProbe.currentPhysFootprintBytes()) / (1024.0 * 1024.0)
        let selfTest = GFSystemProbe.verifyCppCoreSelfTest()
        let bootSec = GFSystemProbe.currentMachBootTimeSeconds()
        FileLogger.shared.log(String(format: "  Initial phys_footprint: %.2f MB", initialMB))
        FileLogger.shared.log(String(format: "  Mach boot time: %.6f s", bootSec))
        FileLogger.shared.log("  C++17 Quat Swing-Twist self-test: \(selfTest)")
    }

    // MARK: - Probe 2: Metal A7 Zero-Copy YUV Check
    private func probe2MetalA7Capabilities() {
        FileLogger.shared.log("[2/6] Metal A7 Zero-Copy YUV Check...")
        let info = GFSystemProbe.probeMetalA7Capabilities()
        let orderedKeys = [
            "deviceName",
            "supportsGPUFamily1v5",
            "textureCacheCreated",
            "pixelBufferCreated",
            "lumaTextureMapped",
            "chromaTextureMapped",
            "defaultLibraryLoaded",
            "lumaPipelineCompiled",
            "chromaPipelineCompiled",
            "maxThreadsPerThreadgroup",
            "lumaPipelineError",
            "chromaPipelineError",
            "error"
        ]
        for key in orderedKeys {
            if let val = info[key] {
                FileLogger.shared.log("  \(key): \(val)")
            }
        }
    }

    // MARK: - Probe 3: CoreMotion Gyro Frequency & Clock Domain
    private func probe3CoreMotionFrequencyAndClock() {
        FileLogger.shared.log("[3/6] CoreMotion Gyro (200Hz req) & DeviceMotion (100Hz req) for 3.0s...")
        let manager = CMMotionManager()
        guard manager.isGyroAvailable else {
            FileLogger.shared.log("  FAIL: Gyroscope not available.")
            return
        }

        let opQueue = OperationQueue()
        opQueue.name = "com.gyrostabilizer5s.motion"
        opQueue.maxConcurrentOperationCount = 1

        var gyroTimestamps: [TimeInterval] = []
        var gyroClockOffsets: [Double] = []
        var dmCount = 0

        manager.gyroUpdateInterval = 1.0 / 200.0
        manager.deviceMotionUpdateInterval = 1.0 / 100.0

        let lock = NSLock()
        manager.startGyroUpdates(to: opQueue) { data, error in
            guard let data = data, error == nil else { return }
            let machNow = GFSystemProbe.currentMachBootTimeSeconds()
            lock.lock()
            gyroTimestamps.append(data.timestamp)
            gyroClockOffsets.append(data.timestamp - machNow)
            lock.unlock()
        }

        if manager.isDeviceMotionAvailable {
            manager.startDeviceMotionUpdates(using: .xArbitraryCorrectedZVertical, to: opQueue) { data, error in
                guard data != nil, error == nil else { return }
                lock.lock()
                dmCount += 1
                lock.unlock()
            }
        } else {
            FileLogger.shared.log("  WARN: DeviceMotion not available.")
        }

        Thread.sleep(forTimeInterval: 3.0)
        manager.stopGyroUpdates()
        manager.stopDeviceMotionUpdates()
        opQueue.waitUntilAllOperationsAreFinished()

        lock.lock()
        let samples = gyroTimestamps
        let offsets = gyroClockOffsets
        let dmTotal = dmCount
        lock.unlock()

        if samples.count >= 2, let first = samples.first, let last = samples.last, last > first {
            let duration = last - first
            let hz = Double(samples.count - 1) / duration
            var minDt = Double.greatestFiniteMagnitude
            var maxDt = 0.0
            for i in 1..<samples.count {
                let dt = samples[i] - samples[i - 1]
                if dt < minDt { minDt = dt }
                if dt > maxDt { maxDt = dt }
            }
            let avgOffset = offsets.reduce(0.0, +) / Double(offsets.count)
            FileLogger.shared.log(String(format: "  Gyro samples: %d in %.3fs -> %.2f Hz", samples.count, duration, hz))
            FileLogger.shared.log(String(format: "  Gyro dt min/max: %.3f ms / %.3f ms", minDt * 1000.0, maxDt * 1000.0))
            FileLogger.shared.log(String(format: "  Gyro timestamp - machBootTime avg offset: %.6f s", avgOffset))
        } else {
            FileLogger.shared.log("  FAIL: Insufficient gyro samples (\(samples.count)).")
        }

        FileLogger.shared.log(String(format: "  DeviceMotion samples in 3.0s: %d (~%.1f Hz)", dmTotal, Double(dmTotal) / 3.0))
    }

    // MARK: - Probe 4: Camera 1080p30 & Stabilization Off & PTS Sync
    private func probe4Camera1080p30AndPTS() {
        FileLogger.shared.log("[4/6] Camera 1080p30, Stabilization Off & PTS Sync...")
        let authStatus = AVCaptureDevice.authorizationStatus(for: .video)
        if authStatus == .notDetermined {
            let sem = DispatchSemaphore(value: 0)
            var granted = false
            AVCaptureDevice.requestAccess(for: .video) { ok in
                granted = ok
                sem.signal()
            }
            sem.wait()
            if !granted {
                FileLogger.shared.log("  FAIL: Camera permission denied.")
                return
            }
        } else if authStatus != .authorized {
            FileLogger.shared.log("  FAIL: Camera permission status=\(authStatus.rawValue).")
            return
        }

        guard let device = AVCaptureDevice.default(for: .video) else {
            FileLogger.shared.log("  FAIL: No video capture device found.")
            return
        }

        let session = AVCaptureSession()
        session.beginConfiguration()
        if session.canSetSessionPreset(.hd1920x1080) {
            session.sessionPreset = .hd1920x1080
        } else {
            FileLogger.shared.log("  WARN: Cannot set .hd1920x1080 preset.")
        }

        do {
            let input = try AVCaptureDeviceInput(device: device)
            if session.canAddInput(input) {
                session.addInput(input)
            }
        } catch {
            FileLogger.shared.log("  FAIL: AVCaptureDeviceInput error: \(error.localizedDescription)")
            session.commitConfiguration()
            return
        }

        let output = AVCaptureVideoDataOutput()
        output.videoSettings = [
            kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
        ]
        output.alwaysDiscardsLateVideoFrames = true

        let collector = CameraFrameCollector(targetCount: 30)
        let videoQueue = DispatchQueue(label: "com.gyrostabilizer5s.video")
        output.setSampleBufferDelegate(collector, queue: videoQueue)

        if session.canAddOutput(output) {
            session.addOutput(output)
        }
        session.commitConfiguration()

        do {
            try device.lockForConfiguration()
            let frameDuration = CMTime(value: 1, timescale: 30)
            device.activeVideoMinFrameDuration = frameDuration
            device.activeVideoMaxFrameDuration = frameDuration

            let focusLockSupported = device.isFocusModeSupported(.locked)
            let customExposureSupported = device.isExposureModeSupported(.custom)
            let lensPosSupported = device.isLockingFocusWithCustomLensPositionSupported
            FileLogger.shared.log("  FocusLocked=\(focusLockSupported), CustomExposure=\(customExposureSupported), CustomLensPos=\(lensPosSupported)")
            device.unlockForConfiguration()
        } catch {
            FileLogger.shared.log("  WARN: lockForConfiguration error: \(error.localizedDescription)")
        }

        if let conn = output.connection(with: .video) {
            let stabSupported = conn.isVideoStabilizationSupported
            if stabSupported {
                conn.preferredVideoStabilizationMode = .off
            }
            FileLogger.shared.log("  VideoStabilizationSupported=\(stabSupported), activeMode=\(conn.activeVideoStabilizationMode.rawValue)")
        }

        session.startRunning()
        _ = collector.semaphore.wait(timeout: .now() + 5.0)
        session.stopRunning()

        let offsets = collector.ptsMinusMachOffsets
        if offsets.isEmpty {
            FileLogger.shared.log("  FAIL: Captured 0 frames.")
        } else {
            let minOff = offsets.min() ?? 0
            let maxOff = offsets.max() ?? 0
            let avgOff = offsets.reduce(0.0, +) / Double(offsets.count)
            FileLogger.shared.log(String(format: "  Captured %d frames; PTS - machBootTime avg=%.6f s (min=%.6f, max=%.6f)",
                                         offsets.count, avgOff, minOff, maxOff))
        }
    }

    // MARK: - Probe 5: H.264 Hardware Encoder Max Bitrate Probe
    private func probe5H264MaxBitrate() {
        FileLogger.shared.log("[5/6] H.264 Hardware Encoder Bitrate Probe (1080p30, 15 frames)...")
        let bitrates = [25_000_000, 35_000_000, 50_000_000, 65_000_000]

        for bitrate in bitrates {
            let mbps = bitrate / 1_000_000
            let tmpURL = URL(fileURLWithPath: NSTemporaryDirectory())
                .appendingPathComponent("probe_enc_\(mbps)mbps.mp4")
            try? FileManager.default.removeItem(at: tmpURL)

            do {
                let writer = try AVAssetWriter(outputURL: tmpURL, fileType: .mp4)
                let compressionProps: [String: Any] = [
                    AVVideoAverageBitRateKey: bitrate,
                    AVVideoProfileLevelKey: AVVideoProfileLevelH264HighAutoLevel,
                    AVVideoExpectedSourceFrameRateKey: 30
                ]
                let videoSettings: [String: Any] = [
                    AVVideoCodecKey: AVVideoCodecType.h264,
                    AVVideoWidthKey: 1920,
                    AVVideoHeightKey: 1080,
                    AVVideoCompressionPropertiesKey: compressionProps
                ]

                let input = AVAssetWriterInput(mediaType: .video, outputSettings: videoSettings)
                input.expectsMediaDataInRealTime = false

                let adaptorAttrs: [String: Any] = [
                    kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_420YpCbCr8BiPlanarFullRange,
                    kCVPixelBufferWidthKey as String: 1920,
                    kCVPixelBufferHeightKey as String: 1080
                ]
                let adaptor = AVAssetWriterInputPixelBufferAdaptor(
                    assetWriterInput: input,
                    sourcePixelBufferAttributes: adaptorAttrs
                )

                guard writer.canAdd(input) else {
                    FileLogger.shared.log("  \(mbps) Mbps: FAIL (writer.canAdd == false)")
                    continue
                }
                writer.add(input)
                guard writer.startWriting() else {
                    FileLogger.shared.log("  \(mbps) Mbps: FAIL (startWriting: \(writer.error?.localizedDescription ?? "unknown"))")
                    continue
                }
                writer.startSession(atSourceTime: .zero)

                var encodeFailed = false
                for frameIdx in 0..<15 {
                    while !input.isReadyForMoreMediaData {
                        Thread.sleep(forTimeInterval: 0.002)
                    }
                    guard let pool = adaptor.pixelBufferPool else {
                        encodeFailed = true
                        break
                    }
                    var pb: CVPixelBuffer?
                    let status = CVPixelBufferPoolCreatePixelBuffer(kCFAllocatorDefault, pool, &pb)
                    guard status == kCVReturnSuccess, let pixelBuffer = pb else {
                        encodeFailed = true
                        break
                    }
                    // Fill luma with a simple pattern so encoder does real work
                    CVPixelBufferLockBaseAddress(pixelBuffer, [])
                    if let base = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0) {
                        let bytesPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0)
                        memset(base, Int32((frameIdx * 17) & 0xFF), bytesPerRow * 1080)
                    }
                    CVPixelBufferUnlockBaseAddress(pixelBuffer, [])

                    let pts = CMTime(value: Int64(frameIdx), timescale: 30)
                    if !adaptor.append(pixelBuffer, withPresentationTime: pts) {
                        encodeFailed = true
                        break
                    }
                }

                input.markAsFinished()
                let sem = DispatchSemaphore(value: 0)
                writer.finishWriting {
                    sem.signal()
                }
                sem.wait()

                if !encodeFailed && writer.status == .completed {
                    let attrs = try? FileManager.default.attributesOfItem(atPath: tmpURL.path)
                    let bytes = (attrs?[.size] as? NSNumber)?.doubleValue ?? 0.0
                    let achievedMbps = (bytes * 8.0) / 0.5 / 1_000_000.0
                    FileLogger.shared.log(String(format: "  %d Mbps: PASS (file=%.1f KB, ~%.1f Mbps)", mbps, bytes / 1024.0, achievedMbps))
                } else {
                    FileLogger.shared.log("  \(mbps) Mbps: FAIL (\(writer.error?.localizedDescription ?? "status=\(writer.status.rawValue)"))")
                }
            } catch {
                FileLogger.shared.log("  \(mbps) Mbps: FAIL (\(error.localizedDescription))")
            }

            try? FileManager.default.removeItem(at: tmpURL)
        }
    }

    // MARK: - Probe 6: Photos Original Asset Check & Safe RAM Ceiling
    private func probe6PhotosAndSafeRamCeiling() {
        FileLogger.shared.log("[6/6] Photos Authorization & Safe RAM Ceiling Probe...")

        let currentStatus = PHPhotoLibrary.authorizationStatus()
        if currentStatus == .notDetermined {
            let sem = DispatchSemaphore(value: 0)
            PHPhotoLibrary.requestAuthorization { newStatus in
                FileLogger.shared.log("  PHPhotoLibrary authorization -> \(newStatus.rawValue)")
                sem.signal()
            }
            sem.wait()
        } else {
            FileLogger.shared.log("  PHPhotoLibrary authorization status -> \(currentStatus.rawValue)")
        }

        var receivedMemWarning = false
        let warningLock = NSLock()
        let observer = NotificationCenter.default.addObserver(
            forName: UIApplication.didReceiveMemoryWarningNotification,
            object: nil,
            queue: nil
        ) { _ in
            warningLock.lock()
            receivedMemWarning = true
            warningLock.unlock()
            FileLogger.shared.log("  WARN: didReceiveMemoryWarningNotification fired!")
        }

        let chunkSize = 32 * 1024 * 1024 // 32 MB
        var allocations: [UnsafeMutableRawPointer] = []
        var peakMB = Double(GFSystemProbe.currentPhysFootprintBytes()) / (1024.0 * 1024.0)

        for step in 1...10 {
            warningLock.lock()
            let shouldStop = receivedMemWarning
            warningLock.unlock()
            if shouldStop {
                FileLogger.shared.log("  Stopping RAM probe early at step \(step - 1) due to memory warning.")
                break
            }

            guard let ptr = malloc(chunkSize) else {
                FileLogger.shared.log("  malloc(32MB) returned nil at step \(step).")
                break
            }
            // Touch pages so iOS counts them toward phys_footprint
            memset(ptr, 0xA5, chunkSize)
            allocations.append(ptr)

            let currentMB = Double(GFSystemProbe.currentPhysFootprintBytes()) / (1024.0 * 1024.0)
            if currentMB > peakMB {
                peakMB = currentMB
            }
            FileLogger.shared.log(String(format: "  Allocated +%d MB total -> phys_footprint=%.1f MB", step * 32, currentMB))
            Thread.sleep(forTimeInterval: 0.15)
        }

        for ptr in allocations {
            free(ptr)
        }
        allocations.removeAll()
        NotificationCenter.default.removeObserver(observer)

        let postFreeMB = Double(GFSystemProbe.currentPhysFootprintBytes()) / (1024.0 * 1024.0)
        FileLogger.shared.log(String(format: "  RAM Probe Summary: peak=%.1f MB, post-free=%.1f MB, warning=%@",
                                     peakMB, postFreeMB, receivedMemWarning ? "YES" : "NO"))
    }
}

private final class CameraFrameCollector: NSObject, AVCaptureVideoDataOutputSampleBufferDelegate {
    private let targetCount: Int
    private let lock = NSLock()
    private var finished = false

    let semaphore = DispatchSemaphore(value: 0)
    private(set) var ptsMinusMachOffsets: [Double] = []

    init(targetCount: Int) {
        self.targetCount = targetCount
    }

    func captureOutput(
        _ output: AVCaptureOutput,
        didOutput sampleBuffer: CMSampleBuffer,
        from connection: AVCaptureConnection
    ) {
        let machNow = GFSystemProbe.currentMachBootTimeSeconds()
        let ptsSec = CMTimeGetSeconds(CMSampleBufferGetPresentationTimeStamp(sampleBuffer))

        lock.lock()
        defer { lock.unlock() }
        guard !finished else { return }

        ptsMinusMachOffsets.append(ptsSec - machNow)
        if ptsMinusMachOffsets.count >= targetCount {
            finished = true
            semaphore.signal()
        }
    }
}
