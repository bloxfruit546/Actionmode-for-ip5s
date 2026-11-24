import AVFoundation
import CoreMotion
import CoreVideo
import Foundation

enum CaptureMode: UInt32 {
    case cinema1080p30 = 0
    case highSpeed720p120 = 1
    case action720p60 = 2

    var width: Int32 {
        switch self {
        case .cinema1080p30: return 1920
        case .highSpeed720p120, .action720p60: return 1280
        }
    }

    var height: Int32 {
        switch self {
        case .cinema1080p30: return 1080
        case .highSpeed720p120, .action720p60: return 720
        }
    }

    var nominalFps: Double {
        switch self {
        case .cinema1080p30: return 30.0
        case .action720p60: return 60.0
        case .highSpeed720p120: return 120.0
        }
    }

    var displayName: String {
        switch self {
        case .cinema1080p30: return "1080p30"
        case .action720p60: return "720p60"
        case .highSpeed720p120: return "720p120"
        }
    }
}

enum ShutterCeilingMode: Int32 {
    case autoOff = 0
    case indoor120 = 120
    case action200 = 200
    case outdoor400 = 400
}

enum BitratePreset: Int {
    case mbps35 = 35_000_000
    case mbps45 = 45_000_000
    case mbps50 = 50_000_000
    case mbps60 = 60_000_000

    var label: String {
        switch self {
        case .mbps35: return "35M"
        case .mbps45: return "45M"
        case .mbps50: return "50M"
        case .mbps60: return "60M"
        }
    }
}

protocol ProCameraEngineDelegate: AnyObject {
    func cameraEngineDidUpdateStatus(
        eisOff: Bool,
        iso: Float,
        maxIso: Float,
        exposureMs: Double,
        shutterCapDenom: Int32,
        fps: Double,
        lumaAvg: Double,
        isDarkMaxIso: Bool,
        lensPos: Float,
        gyroCount: UInt64,
        frameCount: UInt32,
        recordingDurationSec: Double,
        gyroMagRadSec: Double
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
    private(set) var lockFocusAndWhiteBalanceOnRecord = true
    private(set) var isAntiBlurCapEnabled = true
    private(set) var shutterCapTimescale: Int32 = ShutterCeilingMode.action200.rawValue
    private(set) var isMotoModeEnabled = false
    private(set) var isRecording = false
    private var shutterGuardFrameCounter: Int = 0
    private var latestLumaAvg: Double = 0.44
    private var isDarkMaxIsoReached: Bool = false

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

    // Record-time focus/WB lock & dropped frame counters
    private let dropLock = NSLock()
    private var recordedHardwareDrops: UInt32 = 0
    private var recordedWriterDrops: UInt32 = 0
    private var preRecordFocusWasAuto = false
    private var preRecordWbWasAuto = false
    private var lockedRecLensPos: Float = 0.0
    private var lockedRecWbGains: [Float] = []
    private var didLockOnCurrentRecord = false

    // Circular 500ms IMU pre-roll buffer + live sidecar sink
    private struct BufferedGyroSample {
        let timestamp: Double
        let gx: Double
        let gy: Double
        let gz: Double
    }
    private struct BufferedAccelSample {
        let timestamp: Double
        let ax: Float
        let ay: Float
        let az: Float
    }
    private let imuBufferLock = NSLock()
    private var gyroRingBuffer: [BufferedGyroSample] = []
    private var accelRingBuffer: [BufferedAccelSample] = []
    private var activeSidecarWriter: GFSidecarWriterBridge?
    private var latestGyroMagRadSec: Double = 0.0
    private var isImuRunning = false

    func startSession() {
        sessionQueue.async { [weak self] in
            self?.configureSessionIfNeeded()
            if self?.session.isRunning == false {
                self?.session.startRunning()
            }
            self?.startBackgroundImuStream()
            self?.startTelemetryTimer()
        }
    }

    func stopSession() {
        sessionQueue.async { [weak self] in
            self?.stopTelemetryTimer()
            self?.stopBackgroundImuStream()
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

    func setLockFocusAndWhiteBalanceOnRecord(_ enabled: Bool) {
        sessionQueue.async { [weak self] in
            self?.lockFocusAndWhiteBalanceOnRecord = enabled
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
            self.enforceShutterPriorityCustomAE(forceImmediate: true)
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
            if !locked {
                self.enforceShutterPriorityCustomAE(forceImmediate: true)
            }
        }
    }

    func setShutterCapTimescale(_ timescale: Int32) {
        sessionQueue.async { [weak self] in
            guard let self = self, let dev = self.videoDevice else { return }
            self.shutterCapTimescale = timescale
            self.isAntiBlurCapEnabled = (timescale > 0)
            do {
                try dev.lockForConfiguration()
                if timescale > 0 {
                    dev.activeMaxExposureDuration = CMTime(value: 1, timescale: timescale)
                } else {
                    if dev.isExposureModeSupported(.continuousAutoExposure) && !self.isExposureLocked {
                        dev.exposureMode = .continuousAutoExposure
                    }
                    dev.activeMaxExposureDuration = .invalid
                    self.isDarkMaxIsoReached = false
                }
                dev.unlockForConfiguration()
            } catch {
                FileLogger.shared.log("Shutter cap error: \(error.localizedDescription)")
            }
            if timescale > 0 {
                self.enforceShutterPriorityCustomAE(forceImmediate: true)
            }
        }
    }

    func setAntiBlurShutterCap(_ enabled: Bool) {
        let ts: Int32 = enabled ? (isMotoModeEnabled ? ShutterCeilingMode.outdoor400.rawValue : ShutterCeilingMode.action200.rawValue) : 0
        setShutterCapTimescale(ts)
    }

    func setMotoMode(_ enabled: Bool) {
        sessionQueue.async { [weak self] in
            guard let self = self, let dev = self.videoDevice else { return }
            self.isMotoModeEnabled = enabled
            do {
                try dev.lockForConfiguration()
                if enabled {
                    if dev.isLockingFocusWithCustomLensPositionSupported {
                        dev.setFocusModeLocked(lensPosition: 0.88, completionHandler: nil)
                        self.isFocusLocked = true
                    } else if dev.isFocusModeSupported(.locked) {
                        dev.focusMode = .locked
                        self.isFocusLocked = true
                    }
                    self.shutterCapTimescale = ShutterCeilingMode.outdoor400.rawValue
                    dev.activeMaxExposureDuration = CMTime(value: 1, timescale: self.shutterCapTimescale)
                    self.isAntiBlurCapEnabled = true
                    self.currentBitrate = .mbps50
                } else {
                    if dev.isFocusModeSupported(.continuousAutoFocus) {
                        dev.focusMode = .continuousAutoFocus
                        self.isFocusLocked = false
                    }
                    let capTimescale = ShutterCeilingMode.action200.rawValue
                    self.shutterCapTimescale = self.isAntiBlurCapEnabled ? capTimescale : 0
                    dev.activeMaxExposureDuration = self.isAntiBlurCapEnabled ? CMTime(value: 1, timescale: capTimescale) : .invalid
                    self.currentBitrate = .mbps45
                }
                dev.unlockForConfiguration()
            } catch {
                FileLogger.shared.log("Moto Mode error: \(error.localizedDescription)")
            }
            self.enforceShutterPriorityCustomAE(forceImmediate: true)
        }
    }

    func startRecording() {
        sessionQueue.async { [weak self] in
            guard let self = self, !self.isRecording else { return }

            self.dropLock.lock()
            self.recordedHardwareDrops = 0
            self.recordedWriterDrops = 0
            self.dropLock.unlock()

            self.preRecordFocusWasAuto = false
            self.preRecordWbWasAuto = false
            self.lockedRecLensPos = self.videoDevice?.lensPosition ?? 0.0
            self.lockedRecWbGains = []
            self.didLockOnCurrentRecord = false

            if self.lockFocusAndWhiteBalanceOnRecord, let dev = self.videoDevice {
                do {
                    try dev.lockForConfiguration()
                    if !self.isFocusLocked && dev.focusMode != .locked {
                        let curLens = dev.lensPosition
                        if dev.isLockingFocusWithCustomLensPositionSupported {
                            dev.setFocusModeLocked(lensPosition: curLens, completionHandler: nil)
                            self.preRecordFocusWasAuto = true
                            self.lockedRecLensPos = curLens
                        } else if dev.isFocusModeSupported(.locked) {
                            dev.focusMode = .locked
                            self.preRecordFocusWasAuto = true
                            self.lockedRecLensPos = curLens
                        }
                    } else {
                        self.lockedRecLensPos = dev.lensPosition
                    }
                    if dev.whiteBalanceMode != .locked && dev.isWhiteBalanceModeSupported(.locked) {
                        let rawGains = dev.deviceWhiteBalanceGains
                        let maxG = max(1.0, dev.maxWhiteBalanceGain)
                        let clamped = AVCaptureDevice.WhiteBalanceGains(
                            redGain: min(maxG, max(1.0, rawGains.redGain)),
                            greenGain: min(maxG, max(1.0, rawGains.greenGain)),
                            blueGain: min(maxG, max(1.0, rawGains.blueGain))
                        )
                        dev.setWhiteBalanceModeLocked(with: clamped, completionHandler: nil)
                        self.preRecordWbWasAuto = true
                        self.lockedRecWbGains = [clamped.redGain, clamped.greenGain, clamped.blueGain]
                    }
                    dev.unlockForConfiguration()
                    self.didLockOnCurrentRecord = true
                    FileLogger.shared.log(String(
                        format: "Record Locks ACTIVE: lensPos=%.3f wbGains=(%.2f,%.2f,%.2f)",
                        self.lockedRecLensPos,
                        self.lockedRecWbGains.count == 3 ? self.lockedRecWbGains[0] : 0.0,
                        self.lockedRecWbGains.count == 3 ? self.lockedRecWbGains[1] : 0.0,
                        self.lockedRecWbGains.count == 3 ? self.lockedRecWbGains[2] : 0.0
                    ))
                } catch {
                    FileLogger.shared.log("Record Focus/WB lock error: \(error.localizedDescription)")
                }
            }

            let paths = CaptureTakeStore.makeNewTakePaths()
            let calib = CalibrationManager.shared.current
            let mode = self.currentMode
            let bitrate = (mode == .action720p60 && self.currentBitrate == .mbps45) ? 35_000_000 : self.currentBitrate.rawValue

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
                let fpsInt = Int(mode.nominalFps)
                let compressionProps: [String: Any] = [
                    AVVideoAverageBitRateKey: bitrate,
                    AVVideoProfileLevelKey: AVVideoProfileLevelH264HighAutoLevel,
                    AVVideoH264EntropyModeKey: AVVideoH264EntropyModeCABAC,
                    AVVideoMaxKeyFrameIntervalKey: max(1, fpsInt / 2),
                    AVVideoExpectedSourceFrameRateKey: fpsInt
                ]
                let videoSettings: [String: Any] = [
                    AVVideoCodecKey: AVVideoCodecType.h264,
                    AVVideoWidthKey: Int(mode.width),
                    AVVideoHeightKey: Int(mode.height),
                    AVVideoCompressionPropertiesKey: compressionProps
                ]
                let vInput = AVAssetWriterInput(mediaType: .video, outputSettings: videoSettings)
                vInput.expectsMediaDataInRealTime = true
                vInput.transform = self.computeInitialVideoTransform()

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

                // Flush 500ms IMU pre-roll buffer and attach live IMU sink BEFORE enabling recording
                self.imuBufferLock.lock()
                for g in self.gyroRingBuffer {
                    _ = scWriter.writeGyroTimestamp(g.timestamp, gx: g.gx, gy: g.gy, gz: g.gz)
                    self.recordedGyroCount += 1
                }
                for a in self.accelRingBuffer {
                    _ = scWriter.writeAccelTimestamp(a.timestamp, ax: a.ax, ay: a.ay, az: a.az)
                }
                self.activeSidecarWriter = scWriter
                self.imuBufferLock.unlock()

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

            if let dev = self.videoDevice, (self.preRecordFocusWasAuto || self.preRecordWbWasAuto) {
                do {
                    try dev.lockForConfiguration()
                    if self.preRecordFocusWasAuto && !self.isFocusLocked && !self.isMotoModeEnabled && dev.isFocusModeSupported(.continuousAutoFocus) {
                        dev.focusMode = .continuousAutoFocus
                    }
                    if self.preRecordWbWasAuto && dev.isWhiteBalanceModeSupported(.continuousAutoWhiteBalance) {
                        dev.whiteBalanceMode = .continuousAutoWhiteBalance
                    }
                    dev.unlockForConfiguration()
                } catch {
                    FileLogger.shared.log("Record Focus/WB restore error: \(error.localizedDescription)")
                }
            }
            self.preRecordFocusWasAuto = false
            self.preRecordWbWasAuto = false

            // Post-roll 120ms so final IMU timestamps always cover lastFramePts + 0.12s
            Thread.sleep(forTimeInterval: 0.12)

            self.imuBufferLock.lock()
            self.activeSidecarWriter = nil
            self.imuBufferLock.unlock()

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

                    self.dropLock.lock()
                    let hwDrops = self.recordedHardwareDrops
                    let wrDrops = self.recordedWriterDrops
                    self.dropLock.unlock()

                    let auxMeta = CaptureTakeAuxMetadata(
                        hardwareDroppedFrames: hwDrops,
                        writerDroppedFrames: wrDrops,
                        lockedLensPosition: self.lockedRecLensPos,
                        lockedWhiteBalanceGains: self.lockedRecWbGains,
                        focusAndWbLockedOnRecord: self.didLockOnCurrentRecord
                    )
                    CaptureTakeStore.saveAuxMetadata(auxMeta, for: paths.uuid)

                    self.assetWriter = nil
                    self.videoWriterInput = nil
                    self.audioWriterInput = nil
                    self.pixelBufferAdaptor = nil
                    self.sidecarWriter = nil
                    self.activeTakePaths = nil

                    FileLogger.shared.log(String(
                        format: "Recording STOPPED: %@ | frames=%llu gyro=%.1fHz ptsDrops=%u hwDidDrop=%u writerDrop=%u syncOK=%@",
                        paths.uuid, summary.frameCount, summary.gyroHz, summary.droppedFrames,
                        hwDrops, wrDrops, summary.syncCoverageOk ? "YES" : "NO"
                    ))
                    if hwDrops > 0 || wrDrops > 0 || summary.droppedFrames > 0 {
                        FileLogger.shared.log("WARN: Frame drops detected during \(paths.uuid) (didDrop=\(hwDrops), writerDrop=\(wrDrops), ptsGaps=\(summary.droppedFrames))")
                    }

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
        var bestMaxFps: Double = 0.0
        for format in device.formats {
            let desc = format.formatDescription
            let dims = CMVideoFormatDescriptionGetDimensions(desc)
            if dims.width == targetW && dims.height == targetH {
                let matchingMaxFps = format.videoSupportedFrameRateRanges
                    .filter { $0.maxFrameRate >= targetFps && $0.minFrameRate <= targetFps }
                    .map { $0.maxFrameRate }
                    .max() ?? 0.0
                if matchingMaxFps > 0.0 {
                    let subType = CMFormatDescriptionGetMediaSubType(desc)
                    let isFullRange = (subType == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange)
                    // For 720p60/120, prefer >=120fps format for 7.4ms fast readout; break on exact FullRange match
                    if chosenFormat == nil || matchingMaxFps > bestMaxFps || (matchingMaxFps == bestMaxFps && isFullRange) {
                        chosenFormat = format
                        bestMaxFps = matchingMaxFps
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

            if isAntiBlurCapEnabled && shutterCapTimescale > 0 {
                device.activeMaxExposureDuration = CMTime(value: 1, timescale: shutterCapTimescale)
            } else {
                device.activeMaxExposureDuration = .invalid
            }
            device.unlockForConfiguration()
        } catch {
            FileLogger.shared.log("applyFormat error: \(error.localizedDescription)")
        }
        enforceShutterPriorityCustomAE(forceImmediate: true)

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

    // MARK: - Background Raw IMU Stream (500ms Pre-roll Ring Buffer, NO DeviceMotion)
    private func startBackgroundImuStream() {
        guard !isImuRunning else { return }
        isImuRunning = true

        motionManager.gyroUpdateInterval = 1.0 / 120.0
        motionManager.accelerometerUpdateInterval = 1.0 / 120.0

        if motionManager.isGyroAvailable {
            motionManager.startGyroUpdates(to: motionOpQueue) { [weak self] data, error in
                guard let self = self, let data = data, error == nil else { return }
                let ts = data.timestamp
                let sample = BufferedGyroSample(
                    timestamp: ts,
                    gx: data.rotationRate.x,
                    gy: data.rotationRate.y,
                    gz: data.rotationRate.z
                )

                self.imuBufferLock.lock()
                self.gyroRingBuffer.append(sample)
                self.latestGyroMagRadSec = sqrt(sample.gx * sample.gx + sample.gy * sample.gy + sample.gz * sample.gz)
                let cutoff = ts - 0.55
                while let first = self.gyroRingBuffer.first, first.timestamp < cutoff {
                    self.gyroRingBuffer.removeFirst()
                }
                if let writer = self.activeSidecarWriter {
                    _ = writer.writeGyroTimestamp(ts, gx: sample.gx, gy: sample.gy, gz: sample.gz)
                    self.recordedGyroCount += 1
                }
                self.imuBufferLock.unlock()
            }
        }

        if motionManager.isAccelerometerAvailable {
            motionManager.startAccelerometerUpdates(to: motionOpQueue) { [weak self] data, error in
                guard let self = self, let data = data, error == nil else { return }
                let ts = data.timestamp
                let sample = BufferedAccelSample(
                    timestamp: ts,
                    ax: Float(data.acceleration.x),
                    ay: Float(data.acceleration.y),
                    az: Float(data.acceleration.z)
                )

                self.imuBufferLock.lock()
                self.accelRingBuffer.append(sample)
                let cutoff = ts - 0.55
                while let first = self.accelRingBuffer.first, first.timestamp < cutoff {
                    self.accelRingBuffer.removeFirst()
                }
                if let writer = self.activeSidecarWriter {
                    _ = writer.writeAccelTimestamp(ts, ax: sample.ax, ay: sample.ay, az: sample.az)
                }
                self.imuBufferLock.unlock()
            }
        }
    }

    private func stopBackgroundImuStream() {
        guard isImuRunning else { return }
        isImuRunning = false
        motionManager.stopGyroUpdates()
        motionManager.stopAccelerometerUpdates()
        motionOpQueue.waitUntilAllOperationsAreFinished()
        imuBufferLock.lock()
        gyroRingBuffer.removeAll()
        accelRingBuffer.removeAll()
        imuBufferLock.unlock()
    }

    // MARK: - Orientation Transform & Hardware Shutter Guard Loop
    private func computeInitialVideoTransform() -> CGAffineTransform {
        imuBufferLock.lock()
        let lastAccel = accelRingBuffer.last
        imuBufferLock.unlock()
        guard let a = lastAccel else {
            return .identity
        }
        // Native buffer is .landscapeRight (1920x1080). Set display transform only (never rotate ISP pixels!).
        if abs(a.ay) >= abs(a.ax) {
            return (a.ay < 0.0)
                ? CGAffineTransform(rotationAngle: .pi / 2.0)
                : CGAffineTransform(rotationAngle: -.pi / 2.0)
        } else {
            return (a.ax > 0.0)
                ? CGAffineTransform(rotationAngle: .pi)
                : .identity
        }
    }

    private func enforceShutterPriorityCustomAE(forceImmediate: Bool) {
        let capTimescale = shutterCapTimescale
        guard isAntiBlurCapEnabled, capTimescale > 0, !isExposureLocked, let dev = videoDevice else { return }
        guard dev.isExposureModeSupported(.custom) else { return }
        if !forceImmediate && dev.isAdjustingExposure {
            return
        }

        let ceilingSec = 1.0 / Double(capTimescale)
        let minISO = dev.activeFormat.minISO
        let maxISO = dev.activeFormat.maxISO
        let hwMinDur = max(CMTimeGetSeconds(dev.activeFormat.minExposureDuration), 1.0 / 4000.0)
        let curDur = min(ceilingSec, max(hwMinDur, CMTimeGetSeconds(dev.exposureDuration)))
        let curISO = min(maxISO, max(minISO, dev.iso))

        var newDur = curDur
        var newISO = curISO

        if forceImmediate || dev.exposureMode != .custom {
            // Lock .custom immediately before frame 0 so 0.00-0.25s NEVER falls back to 1/30s Auto Exposure
            let evTotal = curDur * Double(curISO)
            if evTotal <= ceilingSec * Double(minISO) {
                newISO = minISO
                newDur = min(ceilingSec, max(hwMinDur, evTotal / Double(minISO)))
            } else {
                newDur = ceilingSec
                newISO = min(maxISO, max(minISO, Float(evTotal / ceilingSec)))
            }
        } else {
            let yAvg = max(0.02, min(0.98, latestLumaAvg))
            let lumaErrEv = log2(0.44 / yAvg)
            let hwOffsetEv = -Double(dev.exposureTargetOffset)
            let evNeed = 0.45 * lumaErrEv + 0.55 * hwOffsetEv

            if abs(evNeed) < 0.08 {
                return
            }

            let gainFactor = pow(2.0, evNeed * 0.35)
            let evTotal = curDur * Double(curISO) * gainFactor
            if evTotal <= ceilingSec * Double(minISO) {
                newISO = minISO
                newDur = min(ceilingSec, max(hwMinDur, evTotal / Double(minISO)))
            } else {
                newDur = ceilingSec
                newISO = min(maxISO, max(minISO, Float(evTotal / ceilingSec)))
            }
        }

        isDarkMaxIsoReached = (newISO >= maxISO * 0.96 && newDur >= ceilingSec * 0.98)

        do {
            try dev.lockForConfiguration()
            dev.setExposureModeCustom(
                duration: CMTimeMakeWithSeconds(newDur, preferredTimescale: 1_000_000),
                iso: newISO,
                completionHandler: nil
            )
            dev.unlockForConfiguration()
        } catch {}
    }

    private func computeCenterWeightedLuma(_ pixelBuffer: CVPixelBuffer) -> Double {
        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }
        guard let base = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0) else { return 0.44 }
        let w = CVPixelBufferGetWidthOfPlane(pixelBuffer, 0)
        let h = CVPixelBufferGetHeightOfPlane(pixelBuffer, 0)
        let stride = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0)
        let ptr = base.assumingMemoryBound(to: UInt8.self)

        let x0 = w / 6, x1 = (w * 5) / 6
        let y0 = h / 6, y1 = (h * 5) / 6
        var sum: Double = 0.0
        var count: Double = 0.0
        var y = y0
        while y < y1 {
            let row = ptr + y * stride
            var x = x0
            while x < x1 {
                sum += Double(row[x])
                count += 1.0
                x += 16
            }
            y += 16
        }
        return (count > 0.0) ? (sum / (count * 255.0)) : 0.44
    }

    private func startTelemetryTimer() {
        stopTelemetryTimer()
        let timer = DispatchSource.makeTimerSource(queue: sessionQueue)
        timer.schedule(deadline: .now() + 0.2, repeating: 0.25)
        timer.setEventHandler { [weak self] in
            guard let self = self else { return }
            self.enforceShutterPriorityCustomAE(forceImmediate: false)
            let iso = self.videoDevice?.iso ?? 0
            let maxIso = self.videoDevice?.activeFormat.maxISO ?? 1600
            let expMs = self.videoDevice.map { CMTimeGetSeconds($0.exposureDuration) * 1000.0 } ?? 0.0
            let capDenom = self.isAntiBlurCapEnabled ? self.shutterCapTimescale : 0
            let fps = self.currentMode.nominalFps
            let luma = self.latestLumaAvg
            let darkMax = self.isDarkMaxIsoReached
            let lens = self.videoDevice?.lensPosition ?? 0
            let eis = self.isEISOff

            var dur = 0.0
            var frames: UInt32 = 0
            var gyros: UInt64 = 0
            self.imuBufferLock.lock()
            let gMag = self.latestGyroMagRadSec
            self.imuBufferLock.unlock()
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
                    maxIso: maxIso,
                    exposureMs: expMs,
                    shutterCapDenom: capDenom,
                    fps: fps,
                    lumaAvg: luma,
                    isDarkMaxIso: darkMax,
                    lensPos: lens,
                    gyroCount: gyros,
                    frameCount: frames,
                    recordingDurationSec: dur,
                    gyroMagRadSec: gMag
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
        if output === videoOutput {
            shutterGuardFrameCounter += 1
            if shutterGuardFrameCounter % 4 == 0 && isAntiBlurCapEnabled && !isExposureLocked {
                if let pb = CMSampleBufferGetImageBuffer(sampleBuffer) {
                    latestLumaAvg = computeCenterWeightedLuma(pb)
                }
                sessionQueue.async { [weak self] in
                    self?.enforceShutterPriorityCustomAE(forceImmediate: false)
                }
            }
        }
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
                    } else {
                        self.dropLock.lock()
                        self.recordedWriterDrops += 1
                        let curDrops = self.recordedWriterDrops
                        self.dropLock.unlock()
                        if curDrops <= 5 {
                            FileLogger.shared.log("WARN: AVAssetWriter append dropped frame (total writer drops=\(curDrops))")
                        }
                    }
                } else {
                    self.dropLock.lock()
                    self.recordedWriterDrops += 1
                    let curDrops = self.recordedWriterDrops
                    self.dropLock.unlock()
                    if curDrops <= 5 {
                        FileLogger.shared.log("WARN: videoWriterInput not ready, dropped frame (total writer drops=\(curDrops))")
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

    func captureOutput(
        _ output: AVCaptureOutput,
        didDrop sampleBuffer: CMSampleBuffer,
        from connection: AVCaptureConnection
    ) {
        guard isRecording, output === videoOutput else { return }
        dropLock.lock()
        recordedHardwareDrops += 1
        let count = recordedHardwareDrops
        dropLock.unlock()
        if count <= 10 {
            let reasonAttach = CMGetAttachment(sampleBuffer, key: kCMSampleBufferAttachmentKey_DroppedFrameReason, attachmentModeOut: nil)
            let reasonStr = reasonAttach.map { "\($0)" } ?? "unknown"
            FileLogger.shared.log("WARN: Camera hardware didDrop frame #\(count) (reason=\(reasonStr))")
        }
    }
}
