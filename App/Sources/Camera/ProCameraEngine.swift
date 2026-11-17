import AVFoundation
import CoreMotion
import CoreVideo
import Foundation

enum CaptureMode: UInt32 {
    case cinema1080p30 = 0
    case highSpeed720p120 = 1

    var width: Int32 {
        switch self {
        case .cinema1080p30: return 1920
        case .highSpeed720p120: return 1280
        }
    }

    var height: Int32 {
        switch self {
        case .cinema1080p30: return 1080
        case .highSpeed720p120: return 720
        }
    }

    var nominalFps: Double {
        switch self {
        case .cinema1080p30: return 30.0
        case .highSpeed720p120: return 120.0
        }
    }

    var displayName: String {
        switch self {
        case .cinema1080p30: return "1080p30"
        case .highSpeed720p120: return "720p120"
        }
    }
}

enum BitratePreset: Int {
    case mbps35 = 35_000_000
    case mbps45 = 45_000_000
    case mbps60 = 60_000_000

    var label: String {
        switch self {
        case .mbps35: return "35M"
        case .mbps45: return "45M"
        case .mbps60: return "60M"
        }
    }
}

protocol ProCameraEngineDelegate: AnyObject {
    func cameraEngineDidUpdateStatus(
        eisOff: Bool,
        iso: Float,
        exposureMs: Double,
        lensPos: Float,
        gyroCount: UInt64,
        frameCount: UInt32,
        recordingDurationSec: Double
    )
    func cameraEngineDidFinishRecording(take: CaptureTake?, summary: GFSidecarSummaryObjC?, error: Error?)
}

final class ProCameraEngine: NSObject {
    weak var delegate: ProCameraEngineDelegate?

    let session = AVCaptureSession()

    private let sessionQueue = DispatchQueue(label: "com.gyrostabilizer5s.camera.session")
    private let writerQueue = DispatchQueue(label: "com.gyrostabilizer5s.camera.writer")
    private let videoOutputQueue = DispatchQueue(label: "com.gyrostabilizer5s.camera.video", qos: .userInteractive)
    private let audioOutputQueue = DispatchQueue(label: "com.gyrostabilizer5s.camera.audio", qos: .userInitiated)
    private let motionOpQueue: OperationQueue = {
        let q = OperationQueue()
        q.name = "com.gyrostabilizer5s.camera.motion"
        q.maxConcurrentOperationCount = 1
        return q
    }()

    private var videoDevice: AVCaptureDevice?
    private var videoInput: AVCaptureDeviceInput?
    private var audioInput: AVCaptureDeviceInput?
    private let videoOutput = AVCaptureVideoDataOutput()
    private let audioOutput = AVCaptureAudioDataOutput()
    private let motionManager = CMMotionManager()

    private(set) var currentMode: CaptureMode = .cinema1080p30
    private(set) var currentBitrate: BitratePreset = .mbps45
    private(set) var isFocusLocked = false
    private(set) var isExposureLocked = false
    private(set) var isAntiBlurCapEnabled = true
    private(set) var isRecording = false

    // Active recording state (guarded by writerQueue)
    private var assetWriter: AVAssetWriter?
    private var videoWriterInput: AVAssetWriterInput?
    private var audioWriterInput: AVAssetWriterInput?
    private var pixelBufferAdaptor: AVAssetWriterInputPixelBufferAdaptor?
    private var sidecarWriter: GFSidecarWriterBridge?
    private var activeTakePaths: (uuid: String, movURL: URL, sidecarURL: URL)?
    private var sessionStarted = false
    private var recordingStartPts: CMTime = .invalid
    private var lastFramePts: CMTime = .invalid
    private var recordedFrameIndex: UInt32 = 0
    private var recordedGyroCount: UInt64 = 0
    private var activeReadoutSec: Float = 0.0152
    private var telemetryTimer: DispatchSourceTimer?
    private var isEISOff = true

    func startSession() {
        sessionQueue.async { [weak self] in
            self?.configureSessionIfNeeded()
            if self?.session.isRunning == false {
                self?.session.startRunning()
            }
            self?.startTelemetryTimer()
        }
    }

    func stopSession() {
        sessionQueue.async { [weak self] in
            self?.stopTelemetryTimer()
            if self?.session.isRunning == true {
                self?.session.stopRunning()
            }
        }
    }

    func setCaptureMode(_ mode: CaptureMode, completion: (() -> Void)? = nil) {
        sessionQueue.async { [weak self] in
            guard let self = self, !self.isRecording else {
                DispatchQueue.main.async { completion?() }
                return
            }
            self.currentMode = mode
            self.applyFormatForCurrentMode()
            DispatchQueue.main.async { completion?() }
        }
    }

    func setBitratePreset(_ preset: BitratePreset) {
        sessionQueue.async { [weak self] in
            self?.currentBitrate = preset
        }
    }

    func setFocusLocked(_ locked: Bool) {
        sessionQueue.async { [weak self] in
            guard let self = self, let dev = self.videoDevice else { return }
            do {
                try dev.lockForConfiguration()
                if locked && dev.isLockingFocusWithCustomLensPositionSupported {
                    dev.setFocusModeLocked(lensPosition: dev.lensPosition, completionHandler: nil)
                    self.isFocusLocked = true
                } else if !locked && dev.isFocusModeSupported(.continuousAutoFocus) {
                    dev.focusMode = .continuousAutoFocus
                    self.isFocusLocked = false
                }
                dev.unlockForConfiguration()
            } catch {
                FileLogger.shared.log("Focus lock error: \(error.localizedDescription)")
            }
        }
    }

    func setExposureLocked(_ locked: Bool) {
        sessionQueue.async { [weak self] in
            guard let self = self, let dev = self.videoDevice else { return }
            do {
                try dev.lockForConfiguration()
                if locked && dev.isExposureModeSupported(.locked) {
                    dev.exposureMode = .locked
                    self.isExposureLocked = true
                } else if !locked && dev.isExposureModeSupported(.continuousAutoExposure) {
                    dev.exposureMode = .continuousAutoExposure
                    self.isExposureLocked = false
                }
                dev.unlockForConfiguration()
            } catch {
                FileLogger.shared.log("Exposure lock error: \(error.localizedDescription)")
            }
        }
    }

    func setAntiBlurShutterCap(_ enabled: Bool) {
        sessionQueue.async { [weak self] in
            guard let self = self, let dev = self.videoDevice else { return }
            self.isAntiBlurCapEnabled = enabled
            do {
                try dev.lockForConfiguration()
                if enabled {
                    let capTimescale: Int32 = (self.currentMode == .cinema1080p30) ? 120 : 240
                    dev.activeMaxExposureDuration = CMTime(value: 1, timescale: capTimescale)
                } else {
                    dev.activeMaxExposureDuration = .invalid
                }
                dev.unlockForConfiguration()
            } catch {
                FileLogger.shared.log("Anti-blur cap error: \(error.localizedDescription)")
            }
        }
    }

    func startRecording() {
        sessionQueue.async { [weak self] in
            guard let self = self, !self.isRecording else { return }

            let paths = CaptureTakeStore.makeNewTakePaths()
            let calib = CalibrationManager.shared.current
            let mode = self.currentMode
            let bitrate = self.currentBitrate.rawValue

            let readoutSec: Double
            let fx: Double
            let fy: Double
            let cx: Double
            let cy: Double
            if mode == .cinema1080p30 {
                readoutSec = calib.readout1080p30Sec
                fx = calib.fx1080
                fy = calib.fy1080
                cx = calib.cx1080
                cy = calib.cy1080
            } else {
                readoutSec = calib.readout720p120Sec
                let scale = 1280.0 / 1920.0
                fx = calib.fx1080 * scale
                fy = calib.fy1080 * scale
                cx = calib.cx1080 * scale
                cy = calib.cy1080 * scale
            }

            let scWriter = GFSidecarWriterBridge()
            let scOpened = scWriter.open(
                atPath: paths.sidecarURL.path,
                uuid: paths.uuid,
                captureMode: mode.rawValue,
                width: UInt32(mode.width),
                height: UInt32(mode.height),
                nominalFps: mode.nominalFps,
                readoutTimeSec: readoutSec,
                gyroBiasX: calib.gyroBiasX,
                gyroBiasY: calib.gyroBiasY,
                gyroBiasZ: calib.gyroBiasZ,
                fx: fx,
                fy: fy,
                cx: cx,
                cy: cy,
                k1: calib.k1,
                k2: calib.k2,
                p1: calib.p1,
                p2: calib.p2
            )

            guard scOpened else {
                FileLogger.shared.log("FAIL: Could not open sidecar file \(paths.sidecarURL.lastPathComponent)")
                return
            }

            do {
                let writer = try AVAssetWriter(outputURL: paths.movURL, fileType: .mov)
                let compressionProps: [String: Any] = [
                    AVVideoAverageBitRateKey: bitrate,
                    AVVideoProfileLevelKey: AVVideoProfileLevelH264HighAutoLevel,
                    AVVideoExpectedSourceFrameRateKey: Int(mode.nominalFps)
                ]
                let videoSettings: [String: Any] = [
                    AVVideoCodecKey: AVVideoCodecType.h264,
                    AVVideoWidthKey: Int(mode.width),
                    AVVideoHeightKey: Int(mode.height),
                    AVVideoCompressionPropertiesKey: compressionProps
                ]
                let vInput = AVAssetWriterInput(mediaType: .video, outputSettings: videoSettings)
                vInput.expectsMediaDataInRealTime = true

                let adaptorAttrs: [String: Any] = [
                    kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_420YpCbCr8BiPlanarFullRange,
                    kCVPixelBufferWidthKey as String: Int(mode.width),
                    kCVPixelBufferHeightKey as String: Int(mode.height)
                ]
                let adaptor = AVAssetWriterInputPixelBufferAdaptor(
                    assetWriterInput: vInput,
                    sourcePixelBufferAttributes: adaptorAttrs
                )

                guard writer.canAdd(vInput) else {
                    scWriter.closeWithoutFooter()
                    FileLogger.shared.log("FAIL: AVAssetWriter cannot add video input.")
                    return
                }
                writer.add(vInput)

                var aInput: AVAssetWriterInput?
                if self.audioInput != nil {
                    let audioSettings: [String: Any] = [
                        AVFormatIDKey: kAudioFormatMPEG4AAC,
                        AVNumberOfChannelsKey: 1,
                        AVSampleRateKey: 44100.0,
                        AVEncoderBitRateKey: 128000
                    ]
                    let ai = AVAssetWriterInput(mediaType: .audio, outputSettings: audioSettings)
                    ai.expectsMediaDataInRealTime = true
                    if writer.canAdd(ai) {
                        writer.add(ai)
                        aInput = ai
                    }
                }

                guard writer.startWriting() else {
                    scWriter.closeWithoutFooter()
                    FileLogger.shared.log("FAIL: AVAssetWriter startWriting: \(writer.error?.localizedDescription ?? "unknown")")
                    return
                }

                self.writerQueue.sync {
                    self.assetWriter = writer
                    self.videoWriterInput = vInput
                    self.audioWriterInput = aInput
                    self.pixelBufferAdaptor = adaptor
                    self.sidecarWriter = scWriter
                    self.activeTakePaths = paths
                    self.sessionStarted = false
                    self.recordingStartPts = .invalid
                    self.lastFramePts = .invalid
                    self.recordedFrameIndex = 0
                    self.recordedGyroCount = 0
                    self.activeReadoutSec = Float(readoutSec)
                }

                // Start Raw IMU BEFORE video frames begin appending (no DeviceMotion!)
                self.startRawImuCapture(sidecarWriter: scWriter)
                self.isRecording = true
                FileLogger.shared.log("Recording STARTED: \(paths.uuid) [\(mode.displayName) @ \(self.currentBitrate.label)]")
            } catch {
                scWriter.closeWithoutFooter()
                FileLogger.shared.log("FAIL: AVAssetWriter init error: \(error.localizedDescription)")
            }
        }
    }

    func stopRecording() {
        sessionQueue.async { [weak self] in
            guard let self = self, self.isRecording else { return }
            self.isRecording = false

            self.stopRawImuCapture()

            self.writerQueue.async {
                guard let writer = self.assetWriter,
                      let paths = self.activeTakePaths,
                      let scWriter = self.sidecarWriter else {
                    return
                }

                self.videoWriterInput?.markAsFinished()
                self.audioWriterInput?.markAsFinished()

                let durationSec: Double
                if self.recordingStartPts.isValid && self.lastFramePts.isValid {
                    durationSec = max(0.0, CMTimeGetSeconds(CMTimeSubtract(self.lastFramePts, self.recordingStartPts)))
                } else {
                    durationSec = 0.0
                }

                writer.finishWriting {
                    _ = scWriter.close(withDuration: durationSec)

                    let movAttrs = try? FileManager.default.attributesOfItem(atPath: paths.movURL.path)
                    let scAttrs = try? FileManager.default.attributesOfItem(atPath: paths.sidecarURL.path)
                    let movSize = (movAttrs?[.size] as? NSNumber)?.uint64Value ?? 0
                    let scSize = (scAttrs?[.size] as? NSNumber)?.uint64Value ?? 0

                    let take = CaptureTake(
                        uuid: paths.uuid,
                        movURL: paths.movURL,
                        sidecarURL: paths.sidecarURL,
                        createdAt: Date(),
                        movSizeBytes: movSize,
                        sidecarSizeBytes: scSize
                    )
                    let summary = GFSidecarReaderBridge.inspectFile(atPath: paths.sidecarURL.path)
                    let err = writer.error

                    self.assetWriter = nil
                    self.videoWriterInput = nil
                    self.audioWriterInput = nil
                    self.pixelBufferAdaptor = nil
                    self.sidecarWriter = nil
                    self.activeTakePaths = nil

                    FileLogger.shared.log(String(
                        format: "Recording STOPPED: %@ | frames=%llu gyro=%.1fHz dropped=%u syncOK=%@",
                        paths.uuid, summary.frameCount, summary.gyroHz, summary.droppedFrames,
                        summary.syncCoverageOk ? "YES" : "NO"
                    ))

                    DispatchQueue.main.async {
                        self.delegate?.cameraEngineDidFinishRecording(take: take, summary: summary, error: err)
                    }
                }
            }
        }
    }

    // MARK: - Private Session Setup
    private func configureSessionIfNeeded() {
        guard videoInput == nil else { return }

        session.beginConfiguration()
        defer { session.commitConfiguration() }

        guard let device = AVCaptureDevice.default(for: .video) else {
            FileLogger.shared.log("CameraEngine: No back video device.")
            return
        }
        self.videoDevice = device

        do {
            let vInput = try AVCaptureDeviceInput(device: device)
            if session.canAddInput(vInput) {
                session.addInput(vInput)
                self.videoInput = vInput
            }
        } catch {
            FileLogger.shared.log("CameraEngine: video input error: \(error.localizedDescription)")
            return
        }

        if let audioDev = AVCaptureDevice.default(for: .audio) {
            if let aInput = try? AVCaptureDeviceInput(device: audioDev), session.canAddInput(aInput) {
                session.addInput(aInput)
                self.audioInput = aInput
            }
        }

        videoOutput.videoSettings = [
            kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
        ]
        videoOutput.alwaysDiscardsLateVideoFrames = false
        videoOutput.setSampleBufferDelegate(self, queue: videoOutputQueue)
        if session.canAddOutput(videoOutput) {
            session.addOutput(videoOutput)
        }

        audioOutput.setSampleBufferDelegate(self, queue: audioOutputQueue)
        if session.canAddOutput(audioOutput) {
            session.addOutput(audioOutput)
        }

        applyFormatForCurrentMode()
    }

    private func applyFormatForCurrentMode() {
        guard let device = videoDevice else { return }

        let targetW = currentMode.width
        let targetH = currentMode.height
        let targetFps = currentMode.nominalFps

        var chosenFormat: AVCaptureDevice.Format?
        for format in device.formats {
            let desc = format.formatDescription
            let dims = CMVideoFormatDescriptionGetDimensions(desc)
            if dims.width == targetW && dims.height == targetH {
                let supportsFps = format.videoSupportedFrameRateRanges.contains { range in
                    range.maxFrameRate >= targetFps && range.minFrameRate <= targetFps
                }
                if supportsFps {
                    chosenFormat = format
                    let subType = CMFormatDescriptionGetMediaSubType(desc)
                    if subType == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange {
                        break
                    }
                }
            }
        }

        do {
            try device.lockForConfiguration()
            if let fmt = chosenFormat {
                device.activeFormat = fmt
            } else {
                FileLogger.shared.log("WARN: Exact format \(targetW)x\(targetH)@\(targetFps) not found.")
            }

            let frameDuration = CMTime(value: 1, timescale: CMTimeScale(targetFps))
            device.activeVideoMinFrameDuration = frameDuration
            device.activeVideoMaxFrameDuration = frameDuration

            if isAntiBlurCapEnabled {
                let capTimescale: Int32 = (currentMode == .cinema1080p30) ? 120 : 240
                device.activeMaxExposureDuration = CMTime(value: 1, timescale: capTimescale)
            } else {
                device.activeMaxExposureDuration = .invalid
            }
            device.unlockForConfiguration()
        } catch {
            FileLogger.shared.log("applyFormat error: \(error.localizedDescription)")
        }

        if let conn = videoOutput.connection(with: .video) {
            if conn.isVideoStabilizationSupported {
                conn.preferredVideoStabilizationMode = .off
            }
            if conn.isVideoOrientationSupported {
                conn.videoOrientation = .landscapeRight
            }
            isEISOff = (conn.activeVideoStabilizationMode == .off)
        }
    }

    // MARK: - Raw IMU Capture (120Hz request -> ~90Hz hardware, NO DeviceMotion)
    private func startRawImuCapture(sidecarWriter: GFSidecarWriterBridge) {
        motionManager.gyroUpdateInterval = 1.0 / 120.0
        motionManager.accelerometerUpdateInterval = 1.0 / 120.0

        if motionManager.isGyroAvailable {
            motionManager.startGyroUpdates(to: motionOpQueue) { [weak self] data, error in
                guard let data = data, error == nil else { return }
                _ = sidecarWriter.writeGyroTimestamp(
                    data.timestamp,
                    gx: data.rotationRate.x,
                    gy: data.rotationRate.y,
                    gz: data.rotationRate.z
                )
                self?.recordedGyroCount += 1
            }
        }

        if motionManager.isAccelerometerAvailable {
            motionManager.startAccelerometerUpdates(to: motionOpQueue) { data, error in
                guard let data = data, error == nil else { return }
                _ = sidecarWriter.writeAccelTimestamp(
                    data.timestamp,
                    ax: Float(data.acceleration.x),
                    ay: Float(data.acceleration.y),
                    az: Float(data.acceleration.z)
                )
            }
        }
    }

    private func stopRawImuCapture() {
        motionManager.stopGyroUpdates()
        motionManager.stopAccelerometerUpdates()
        motionOpQueue.waitUntilAllOperationsAreFinished()
    }

    // MARK: - Telemetry Timer
    private func startTelemetryTimer() {
        stopTelemetryTimer()
        let timer = DispatchSource.makeTimerSource(queue: sessionQueue)
        timer.schedule(deadline: .now() + 0.2, repeating: 0.25)
        timer.setEventHandler { [weak self] in
            guard let self = self else { return }
            let iso = self.videoDevice?.iso ?? 0
            let expMs = self.videoDevice.map { CMTimeGetSeconds($0.exposureDuration) * 1000.0 } ?? 0.0
            let lens = self.videoDevice?.lensPosition ?? 0
            let eis = self.isEISOff

            var dur = 0.0
            var frames: UInt32 = 0
            var gyros: UInt64 = 0
            if self.isRecording {
                frames = self.recordedFrameIndex
                gyros = self.recordedGyroCount
                if self.recordingStartPts.isValid && self.lastFramePts.isValid {
                    dur = max(0.0, CMTimeGetSeconds(CMTimeSubtract(self.lastFramePts, self.recordingStartPts)))
                }
            }

            DispatchQueue.main.async {
                self.delegate?.cameraEngineDidUpdateStatus(
                    eisOff: eis,
                    iso: iso,
                    exposureMs: expMs,
                    lensPos: lens,
                    gyroCount: gyros,
                    frameCount: frames,
                    recordingDurationSec: dur
                )
            }
        }
        timer.resume()
        telemetryTimer = timer
    }

    private func stopTelemetryTimer() {
        telemetryTimer?.cancel()
        telemetryTimer = nil
    }
}

extension ProCameraEngine: AVCaptureVideoDataOutputSampleBufferDelegate, AVCaptureAudioDataOutputSampleBufferDelegate {
    func captureOutput(
        _ output: AVCaptureOutput,
        didOutput sampleBuffer: CMSampleBuffer,
        from connection: AVCaptureConnection
    ) {
        guard isRecording else { return }

        if output === videoOutput {
            let meta = GFSidecarReaderBridge.extractFrameMetadata(sampleBuffer, fallbackDevice: videoDevice)
            guard let pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }
            let pts = CMSampleBufferGetPresentationTimeStamp(sampleBuffer)

            writerQueue.async { [weak self] in
                guard let self = self,
                      let writer = self.assetWriter,
                      writer.status == .writing,
                      let vInput = self.videoWriterInput,
                      let adaptor = self.pixelBufferAdaptor,
                      let scWriter = self.sidecarWriter else {
                    return
                }

                if !self.sessionStarted {
                    writer.startSession(atSourceTime: pts)
                    self.sessionStarted = true
                    self.recordingStartPts = pts
                }
                self.lastFramePts = pts

                if vInput.isReadyForMoreMediaData {
                    if adaptor.append(pixelBuffer, withPresentationTime: pts) {
                        _ = scWriter.writeFrameIndex(
                            self.recordedFrameIndex,
                            pts: meta.ptsSec,
                            exposureSec: meta.exposureDurationSec,
                            iso: meta.iso,
                            lensPosition: meta.lensPosition,
                            rollingShutterSec: self.activeReadoutSec
                        )
                        self.recordedFrameIndex += 1
                    }
                }
            }
        } else if output === audioOutput {
            writerQueue.async { [weak self] in
                guard let self = self,
                      self.sessionStarted,
                      let writer = self.assetWriter,
                      writer.status == .writing,
                      let aInput = self.audioWriterInput,
                      aInput.isReadyForMoreMediaData else {
                    return
                }
                aInput.append(sampleBuffer)
            }
        }
    }
}
