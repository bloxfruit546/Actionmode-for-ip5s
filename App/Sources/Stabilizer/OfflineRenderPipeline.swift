import AVFoundation
import CoreVideo
import Foundation
import Photos

struct StabilizeExportConfig: Codable {
    let pitchSmoothness: Double
    let yawSmoothness: Double
    let rollSmoothness: Double
    let velocityDamping: Double
    let timeOffsetMs: Double
    let maxZoomLimit: Double // <= 1.0 means unlimited safe zoom
    let horizonLockEnabled: Bool
    let horizonLockAmount: Double
    let horizonTiltOffsetDeg: Double
    let horizonMode: Int // 0: Off, 1: Action Level (±40°), 2: Full 360° Lock
    let rollingShutterCorrection: Bool
    let dynamicZoomEnabled: Bool
    let axisPresetIndex: Int
    let hasReadoutOverride: Bool
    let signedReadoutTimeMs: Double
    let focalScaleOverride: Double
    let rollGyroScaleOverride: Double
    let hybridResidualEnabled: Bool
    let translational2DStrength: Double
    let motoActionMode: Bool
    let edgeFillMode: Int
    let fovScaleBoost: Double
    let sharpenStrength: Double
    let exportBitrateMbps: Int

    init(
        pitchSmoothness: Double,
        yawSmoothness: Double,
        rollSmoothness: Double,
        velocityDamping: Double,
        timeOffsetMs: Double,
        maxZoomLimit: Double,
        horizonLockEnabled: Bool,
        horizonLockAmount: Double,
        horizonTiltOffsetDeg: Double,
        horizonMode: Int = 1,
        rollingShutterCorrection: Bool,
        dynamicZoomEnabled: Bool,
        axisPresetIndex: Int,
        hasReadoutOverride: Bool,
        signedReadoutTimeMs: Double,
        focalScaleOverride: Double,
        rollGyroScaleOverride: Double = 1.0,
        hybridResidualEnabled: Bool = false,
        translational2DStrength: Double = 0.0,
        motoActionMode: Bool = false,
        edgeFillMode: Int = 0,
        fovScaleBoost: Double = 1.0,
        sharpenStrength: Double = 0.45,
        exportBitrateMbps: Int = 45
    ) {
        self.pitchSmoothness = pitchSmoothness
        self.yawSmoothness = yawSmoothness
        self.rollSmoothness = rollSmoothness
        self.velocityDamping = velocityDamping
        self.timeOffsetMs = timeOffsetMs
        self.maxZoomLimit = maxZoomLimit
        self.horizonLockEnabled = horizonLockEnabled
        self.horizonLockAmount = horizonLockAmount
        self.horizonTiltOffsetDeg = horizonTiltOffsetDeg
        self.horizonMode = horizonMode
        self.rollingShutterCorrection = rollingShutterCorrection
        self.dynamicZoomEnabled = dynamicZoomEnabled
        self.axisPresetIndex = axisPresetIndex
        self.hasReadoutOverride = hasReadoutOverride
        self.signedReadoutTimeMs = signedReadoutTimeMs
        self.focalScaleOverride = focalScaleOverride
        self.rollGyroScaleOverride = rollGyroScaleOverride
        self.hybridResidualEnabled = hybridResidualEnabled
        self.translational2DStrength = translational2DStrength
        self.motoActionMode = motoActionMode
        self.edgeFillMode = edgeFillMode
        self.fovScaleBoost = fovScaleBoost
        self.sharpenStrength = sharpenStrength
        self.exportBitrateMbps = exportBitrateMbps
    }

    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        pitchSmoothness = try c.decode(Double.self, forKey: .pitchSmoothness)
        yawSmoothness = try c.decode(Double.self, forKey: .yawSmoothness)
        rollSmoothness = try c.decode(Double.self, forKey: .rollSmoothness)
        velocityDamping = try c.decode(Double.self, forKey: .velocityDamping)
        timeOffsetMs = try c.decode(Double.self, forKey: .timeOffsetMs)
        maxZoomLimit = try c.decode(Double.self, forKey: .maxZoomLimit)
        horizonLockEnabled = try c.decode(Bool.self, forKey: .horizonLockEnabled)
        horizonLockAmount = try c.decode(Double.self, forKey: .horizonLockAmount)
        horizonTiltOffsetDeg = try c.decode(Double.self, forKey: .horizonTiltOffsetDeg)
        horizonMode = (try? c.decodeIfPresent(Int.self, forKey: .horizonMode)) ?? 1
        rollingShutterCorrection = try c.decode(Bool.self, forKey: .rollingShutterCorrection)
        dynamicZoomEnabled = try c.decode(Bool.self, forKey: .dynamicZoomEnabled)
        axisPresetIndex = try c.decode(Int.self, forKey: .axisPresetIndex)
        hasReadoutOverride = try c.decode(Bool.self, forKey: .hasReadoutOverride)
        signedReadoutTimeMs = try c.decode(Double.self, forKey: .signedReadoutTimeMs)
        focalScaleOverride = try c.decode(Double.self, forKey: .focalScaleOverride)
        rollGyroScaleOverride = (try? c.decodeIfPresent(Double.self, forKey: .rollGyroScaleOverride)) ?? 1.0
        hybridResidualEnabled = (try? c.decodeIfPresent(Bool.self, forKey: .hybridResidualEnabled)) ?? false
        translational2DStrength = (try? c.decodeIfPresent(Double.self, forKey: .translational2DStrength)) ?? 0.0
        motoActionMode = (try? c.decodeIfPresent(Bool.self, forKey: .motoActionMode)) ?? false
        edgeFillMode = (try? c.decodeIfPresent(Int.self, forKey: .edgeFillMode)) ?? 0
        fovScaleBoost = (try? c.decodeIfPresent(Double.self, forKey: .fovScaleBoost)) ?? 1.0
        sharpenStrength = (try? c.decodeIfPresent(Double.self, forKey: .sharpenStrength)) ?? 0.45
        exportBitrateMbps = (try? c.decodeIfPresent(Int.self, forKey: .exportBitrateMbps)) ?? 45
    }
}

struct StabilizeExportResult {
    let outputURL: URL
    let plan: GFStabilizerPlanObjC
    let renderedFrames: Int
    let elapsedSeconds: Double
    let peakRamMB: Double
}

final class OfflineRenderPipeline {
    private let setupQueue = DispatchQueue(label: "com.gyrostab5s.render.setup", qos: .userInitiated)
    private let cancelLock = NSLock()
    private var _isCancelled = false

    private var isCancelled: Bool {
        get {
            cancelLock.lock()
            defer { cancelLock.unlock() }
            return _isCancelled
        }
        set {
            cancelLock.lock()
            _isCancelled = newValue
            cancelLock.unlock()
        }
    }

    func cancel() {
        isCancelled = true
    }

    static var stabilizedDirectory: URL {
        let docs = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
        let dir = docs.appendingPathComponent("Stabilized", isDirectory: true)
        if !FileManager.default.fileExists(atPath: dir.path) {
            try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true, attributes: nil)
        }
        return dir
    }

    func runAutoSync(
        take: CaptureTake,
        motoActionMode: Bool = false,
        completion: @escaping (Result<GFOpticalSyncResultObjC, Error>) -> Void
    ) {
        setupQueue.async { [weak self] in
            guard let self = self else { return }
            let startTime = CFAbsoluteTimeGetCurrent()
            let asset = AVURLAsset(url: take.movURL)
            guard let videoTrack = asset.tracks(withMediaType: .video).first else {
                DispatchQueue.main.async {
                    completion(.failure(self.makeError("No video track found for Auto-Sync.")))
                }
                return
            }

            do {
                let reader = try AVAssetReader(asset: asset)
                let outputSettings: [String: Any] = [
                    kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
                ]
                let trackOut = AVAssetReaderTrackOutput(track: videoTrack, outputSettings: outputSettings)
                trackOut.alwaysCopiesSampleData = false
                guard reader.canAdd(trackOut) else {
                    throw self.makeError("AVAssetReader cannot add video output for Auto-Sync.")
                }
                reader.add(trackOut)
                guard reader.startReading() else {
                    throw self.makeError("AVAssetReader failed to start for Auto-Sync.")
                }

                let collector = GFOpticalSyncCollectorBridge()
                collector.reset()
                var count = 0
                let maxSampleFrames = 110

                while count < maxSampleFrames {
                    var reachedEnd = false
                    autoreleasepool {
                        guard let sample = trackOut.copyNextSampleBuffer() else {
                            reachedEnd = true
                            return
                        }
                        if let pb = CMSampleBufferGetImageBuffer(sample) {
                            let pts = CMTimeGetSeconds(CMSampleBufferGetPresentationTimeStamp(sample))
                            collector.addLumaPixelBuffer(pb, timestampSec: pts)
                            count += 1
                        }
                    }
                    if reachedEnd {
                        break
                    }
                }
                reader.cancelReading()

                let result = collector.solve(withSidecarPath: take.sidecarURL.path, motoActionMode: motoActionMode)
                let elapsed = CFAbsoluteTimeGetCurrent() - startTime
                FileLogger.shared.log(String(format: "Auto-Sync completed in %.2fs (%d frames): %@", elapsed, count, result.diagnosticMessage))

                DispatchQueue.main.async {
                    completion(.success(result))
                }
            } catch {
                DispatchQueue.main.async {
                    completion(.failure(error))
                }
            }
        }
    }

    func computePlanPreview(take: CaptureTake, config: StabilizeExportConfig) -> GFStabilizerPlanObjC? {
        let asset = AVURLAsset(url: take.movURL)
        let rotDeg = asset.tracks(withMediaType: .video).first.map { Self.videoRotationDegrees(from: $0.preferredTransform) } ?? 0
        let bridge = makeBridge(sidecarPath: take.sidecarURL.path, config: config, videoRotationDegrees: rotDeg)
        return bridge?.plan
    }

    static func videoRotationDegrees(from transform: CGAffineTransform) -> Int {
        let angle = atan2(transform.b, transform.a)
        let deg = Int(round(angle * 180.0 / .pi))
        return ((deg % 360) + 360) % 360
    }

    func render(
        take: CaptureTake,
        config: StabilizeExportConfig,
        statusUpdate: ((String) -> Void)? = nil,
        progress: @escaping (_ fraction: Double, _ frameIndex: Int, _ totalFrames: Int, _ ramMB: Double) -> Void,
        completion: @escaping (Result<StabilizeExportResult, Error>) -> Void
    ) {
        isCancelled = false
        setupQueue.async { [weak self] in
            guard let self = self else { return }
            let startTime = CFAbsoluteTimeGetCurrent()

            let asset = AVURLAsset(url: take.movURL)
            guard let videoTrack = asset.tracks(withMediaType: .video).first else {
                DispatchQueue.main.async {
                    completion(.failure(self.makeError("No video track found in .mov.")))
                }
                return
            }
            let rotDeg = Self.videoRotationDegrees(from: videoTrack.preferredTransform)

            guard let bridge = self.makeBridge(sidecarPath: take.sidecarURL.path, config: config, videoRotationDegrees: rotDeg) else {
                DispatchQueue.main.async {
                    completion(.failure(self.makeError("Failed to initialize GFStabilizerBridge.")))
                }
                return
            }

            let plan = bridge.plan
            guard plan.valid else {
                DispatchQueue.main.async {
                    completion(.failure(self.makeError(plan.errorMessage.isEmpty ? "Invalid stabilizer plan." : plan.errorMessage)))
                }
                return
            }

            if config.hybridResidualEnabled && !config.motoActionMode && config.translational2DStrength > 0.01 {
                self.runTranslational2DPass(
                    asset: asset,
                    videoTrack: videoTrack,
                    bridge: bridge,
                    totalFrames: max(1, Int(plan.frameCount)),
                    strength: config.translational2DStrength,
                    statusUpdate: statusUpdate
                )
            }

            let outURL = Self.stabilizedDirectory.appendingPathComponent("\(take.uuid)_stab.mov")
            try? FileManager.default.removeItem(at: outURL)

            let width = Int(plan.width > 0 ? plan.width : UInt32(videoTrack.naturalSize.width))
            let height = Int(plan.height > 0 ? plan.height : UInt32(videoTrack.naturalSize.height))
            let totalFrames = max(1, Int(plan.frameCount))

            do {
                let reader = try AVAssetReader(asset: asset)
                let videoReaderSettings: [String: Any] = [
                    kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_420YpCbCr8BiPlanarFullRange,
                    kCVPixelBufferMetalCompatibilityKey as String: true
                ]
                let videoReaderOut = AVAssetReaderTrackOutput(track: videoTrack, outputSettings: videoReaderSettings)
                videoReaderOut.alwaysCopiesSampleData = false
                guard reader.canAdd(videoReaderOut) else {
                    throw self.makeError("AVAssetReader cannot add video output.")
                }
                reader.add(videoReaderOut)

                var audioReaderOut: AVAssetReaderTrackOutput?
                if let audioTrack = asset.tracks(withMediaType: .audio).first {
                    let aro = AVAssetReaderTrackOutput(track: audioTrack, outputSettings: nil)
                    aro.alwaysCopiesSampleData = false
                    if reader.canAdd(aro) {
                        reader.add(aro)
                        audioReaderOut = aro
                    }
                }

                let writer = try AVAssetWriter(outputURL: outURL, fileType: .mov)
                writer.shouldOptimizeForNetworkUse = false
                if !asset.metadata.isEmpty {
                    writer.metadata = asset.metadata
                } else if !asset.commonMetadata.isEmpty {
                    writer.metadata = asset.commonMetadata
                }

                let targetMbps = max(20, min(60, config.exportBitrateMbps > 0 ? config.exportBitrateMbps : ((height >= 1000) ? 45 : 35)))
                let bitrate = targetMbps * 1_000_000
                let fpsInt = max(1, Int(round(plan.nominalFps > 1 ? plan.nominalFps : 30)))
                let compressionProps: [String: Any] = [
                    AVVideoAverageBitRateKey: bitrate,
                    AVVideoProfileLevelKey: AVVideoProfileLevelH264HighAutoLevel,
                    AVVideoH264EntropyModeKey: AVVideoH264EntropyModeCABAC,
                    AVVideoAllowFrameReorderingKey: true,
                    AVVideoMaxKeyFrameIntervalKey: max(1, fpsInt / 2),
                    AVVideoExpectedSourceFrameRateKey: fpsInt
                ]
                let videoWriterSettings: [String: Any] = [
                    AVVideoCodecKey: AVVideoCodecType.h264,
                    AVVideoWidthKey: width,
                    AVVideoHeightKey: height,
                    AVVideoCompressionPropertiesKey: compressionProps
                ]
                let videoWriterIn = AVAssetWriterInput(mediaType: .video, outputSettings: videoWriterSettings)
                videoWriterIn.expectsMediaDataInRealTime = false
                videoWriterIn.transform = videoTrack.preferredTransform

                let adaptorAttrs: [String: Any] = [
                    kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_420YpCbCr8BiPlanarFullRange,
                    kCVPixelBufferWidthKey as String: width,
                    kCVPixelBufferHeightKey as String: height,
                    kCVPixelBufferMetalCompatibilityKey as String: true
                ]
                let adaptor = AVAssetWriterInputPixelBufferAdaptor(
                    assetWriterInput: videoWriterIn,
                    sourcePixelBufferAttributes: adaptorAttrs
                )

                guard writer.canAdd(videoWriterIn) else {
                    throw self.makeError("AVAssetWriter cannot add video input.")
                }
                writer.add(videoWriterIn)

                var audioWriterIn: AVAssetWriterInput?
                if audioReaderOut != nil {
                    let awi = AVAssetWriterInput(mediaType: .audio, outputSettings: nil)
                    awi.expectsMediaDataInRealTime = false
                    if writer.canAdd(awi) {
                        writer.add(awi)
                        audioWriterIn = awi
                    }
                }

                guard reader.startReading() else {
                    throw self.makeError("AVAssetReader startReading failed: \(reader.error?.localizedDescription ?? "unknown")")
                }
                guard writer.startWriting() else {
                    reader.cancelReading()
                    throw self.makeError("AVAssetWriter startWriting failed: \(writer.error?.localizedDescription ?? "unknown")")
                }
                writer.startSession(atSourceTime: .zero)

                FileLogger.shared.log("Video preferredTransform rotation = \(rotDeg) deg | Applied AxisPreset = \(config.axisPresetIndex) | Applied Offset = \(String(format: "%.1f", config.timeOffsetMs)) ms")
                FileLogger.shared.log(plan.gpuMatrixProof)
                FileLogger.shared.log("OfflineRender STARTED: \(take.uuid) (\(width)x\(height), ~\(totalFrames) frames, RS=\(config.rollingShutterCorrection), HL=\(config.horizonLockEnabled))")

                let videoRenderQueue = DispatchQueue(label: "com.gyrostab5s.render.video", qos: .userInitiated)
                let audioRenderQueue = DispatchQueue(label: "com.gyrostab5s.render.audio", qos: .userInitiated)
                let muxGroup = DispatchGroup()
                let stateLock = NSLock()

                var videoFinished = false
                var audioFinished = (audioReaderOut == nil || audioWriterIn == nil)
                var frameIndex = 0
                var lastLoggedDecile = -1
                var peakRamMB = Double(GFSystemProbe.currentPhysFootprintBytes()) / (1024.0 * 1024.0)
                var renderError: Error?

                let finishVideoOnce: (Error?) -> Void = { err in
                    stateLock.lock()
                    let shouldLeave = !videoFinished
                    if shouldLeave {
                        videoFinished = true
                        if let e = err, renderError == nil {
                            renderError = e
                        }
                    }
                    stateLock.unlock()

                    if shouldLeave {
                        videoWriterIn.markAsFinished()
                        FileLogger.shared.log("OfflineRender: Video track finished (\(frameIndex) frames).")
                        muxGroup.leave()
                    }
                }

                let finishAudioOnce: (Error?) -> Void = { err in
                    stateLock.lock()
                    let shouldLeave = !audioFinished
                    if shouldLeave {
                        audioFinished = true
                        if let e = err, renderError == nil {
                            renderError = e
                        }
                    }
                    stateLock.unlock()

                    if shouldLeave {
                        audioWriterIn?.markAsFinished()
                        FileLogger.shared.log("OfflineRender: Audio track finished.")
                        muxGroup.leave()
                    }
                }

                muxGroup.enter()
                videoWriterIn.requestMediaDataWhenReady(on: videoRenderQueue) { [weak self] in
                    guard let self = self else {
                        finishVideoOnce(nil)
                        return
                    }

                    while videoWriterIn.isReadyForMoreMediaData {
                        if self.isCancelled {
                            reader.cancelReading()
                            finishVideoOnce(self.makeError("Render cancelled by user."))
                            return
                        }

                        var loopError: Error?
                        var reachedEnd = false

                        autoreleasepool {
                            guard let sample = videoReaderOut.copyNextSampleBuffer() else {
                                if reader.status == .failed {
                                    loopError = reader.error ?? self.makeError("AVAssetReader video read failed.")
                                }
                                reachedEnd = true
                                return
                            }

                            guard let srcPB = CMSampleBufferGetImageBuffer(sample) else {
                                return
                            }
                            let pts = CMSampleBufferGetPresentationTimeStamp(sample)

                            guard let pool = adaptor.pixelBufferPool else {
                                loopError = self.makeError("AVAssetWriterInputPixelBufferAdaptor pixelBufferPool is nil.")
                                return
                            }

                            var dstPBOpt: CVPixelBuffer?
                            let pbStatus = CVPixelBufferPoolCreatePixelBuffer(kCFAllocatorDefault, pool, &dstPBOpt)
                            guard pbStatus == kCVReturnSuccess, let dstPB = dstPBOpt else {
                                loopError = self.makeError("CVPixelBufferPoolCreatePixelBuffer failed (status=\(pbStatus)).")
                                return
                            }

                            let warped = bridge.warpPixelBuffer(srcPB, into: dstPB, frameIndex: UInt(frameIndex))
                            if !warped {
                                loopError = self.makeError("Metal warpPixelBuffer failed at frame \(frameIndex).")
                                return
                            }

                            if !adaptor.append(dstPB, withPresentationTime: pts) {
                                let wErr = writer.error?.localizedDescription ?? "unknown writer error"
                                FileLogger.shared.log("OfflineRender ERROR: adaptor.append failed at frame \(frameIndex): \(wErr)")
                                loopError = writer.error ?? self.makeError("AVAssetWriter append failed: \(wErr)")
                                return
                            }

                            frameIndex += 1
                            let ramMB = Double(GFSystemProbe.currentPhysFootprintBytes()) / (1024.0 * 1024.0)
                            if ramMB > peakRamMB {
                                peakRamMB = ramMB
                            }

                            let frac = min(1.0, Double(frameIndex) / Double(totalFrames))
                            let decile = Int(frac * 10.0)
                            if decile > lastLoggedDecile {
                                lastLoggedDecile = decile
                                FileLogger.shared.log(String(format: "OfflineRender: %d%% (%d/%d) | RAM=%.1f MB", decile * 10, frameIndex, totalFrames, ramMB))
                            }

                            if frameIndex % 5 == 0 || frameIndex == totalFrames {
                                let curIdx = frameIndex
                                DispatchQueue.main.async {
                                    progress(frac, curIdx, totalFrames, ramMB)
                                }
                            }
                        }

                        if let err = loopError {
                            reader.cancelReading()
                            finishVideoOnce(err)
                            return
                        }
                        if reachedEnd {
                            finishVideoOnce(nil)
                            return
                        }
                    }
                }

                if let aOut = audioReaderOut, let aIn = audioWriterIn {
                    muxGroup.enter()
                    aIn.requestMediaDataWhenReady(on: audioRenderQueue) { [weak self] in
                        guard let self = self else {
                            finishAudioOnce(nil)
                            return
                        }

                        while aIn.isReadyForMoreMediaData {
                            if self.isCancelled {
                                finishAudioOnce(nil)
                                return
                            }

                            var audioErr: Error?
                            var reachedEnd = false

                            autoreleasepool {
                                guard let aSample = aOut.copyNextSampleBuffer() else {
                                    if reader.status == .failed {
                                        audioErr = reader.error ?? self.makeError("AVAssetReader audio read failed.")
                                    }
                                    reachedEnd = true
                                    return
                                }

                                if !aIn.append(aSample) {
                                    let wErr = writer.error?.localizedDescription ?? "unknown audio append error"
                                    FileLogger.shared.log("OfflineRender ERROR: audio append failed: \(wErr)")
                                    audioErr = writer.error ?? self.makeError("Audio append failed: \(wErr)")
                                    return
                                }
                            }

                            if let err = audioErr {
                                finishAudioOnce(err)
                                return
                            }
                            if reachedEnd {
                                finishAudioOnce(nil)
                                return
                            }
                        }
                    }
                }

                muxGroup.notify(queue: DispatchQueue.global(qos: .userInitiated)) {
                    bridge.flushTextureCache()

                    if self.isCancelled || renderError != nil {
                        reader.cancelReading()
                        writer.cancelWriting()
                        try? FileManager.default.removeItem(at: outURL)
                        let finalErr = renderError ?? self.makeError("Render cancelled by user.")
                        FileLogger.shared.log("OfflineRender ABORTED: \(finalErr.localizedDescription)")
                        DispatchQueue.main.async {
                            completion(.failure(finalErr))
                        }
                        return
                    }

                    writer.finishWriting {
                        if writer.status == .completed {
                            let elapsed = CFAbsoluteTimeGetCurrent() - startTime
                            let res = StabilizeExportResult(
                                outputURL: outURL,
                                plan: plan,
                                renderedFrames: frameIndex,
                                elapsedSeconds: elapsed,
                                peakRamMB: peakRamMB
                            )
                            FileLogger.shared.log(String(
                                format: "OfflineRender DONE: %d frames in %.2fs (%.1f fps) | PeakRAM=%.1f MB | Gain=%.1f%%",
                                frameIndex, elapsed, Double(frameIndex) / max(elapsed, 0.01),
                                peakRamMB, plan.stabilityImprovementPct
                            ))
                            DispatchQueue.main.async {
                                completion(.success(res))
                            }
                        } else {
                            let wErr = writer.error ?? self.makeError("AVAssetWriter finishWriting failed (status=\(writer.status.rawValue)).")
                            try? FileManager.default.removeItem(at: outURL)
                            FileLogger.shared.log("OfflineRender FAILED: \(wErr.localizedDescription)")
                            DispatchQueue.main.async {
                                completion(.failure(wErr))
                            }
                        }
                    }
                }
            } catch {
                try? FileManager.default.removeItem(at: outURL)
                FileLogger.shared.log("OfflineRender SETUP ERROR: \(error.localizedDescription)")
                DispatchQueue.main.async {
                    completion(.failure(error))
                }
            }
        }
    }

    static func saveVideoToPhotos(url: URL, completion: @escaping (Result<Void, Error>) -> Void) {
        PHPhotoLibrary.requestAuthorization { status in
            guard status == .authorized else {
                DispatchQueue.main.async {
                    completion(.failure(NSError(
                        domain: "GyroStabilizer5s",
                        code: -2,
                        userInfo: [NSLocalizedDescriptionKey: "Photo Library permission denied."]
                    )))
                }
                return
            }

            PHPhotoLibrary.shared().performChanges({
                PHAssetChangeRequest.creationRequestForAssetFromVideo(atFileURL: url)
            }) { success, error in
                DispatchQueue.main.async {
                    if success {
                        completion(.success(()))
                    } else {
                        completion(.failure(error ?? NSError(
                            domain: "GyroStabilizer5s",
                            code: -3,
                            userInfo: [NSLocalizedDescriptionKey: "Failed to save video to Photos."]
                        )))
                    }
                }
            }
        }
    }

    private func runTranslational2DPass(
        asset: AVURLAsset,
        videoTrack: AVAssetTrack,
        bridge: GFStabilizerBridge,
        totalFrames: Int,
        strength: Double,
        statusUpdate: ((String) -> Void)?
    ) {
        guard let preReader = try? AVAssetReader(asset: asset) else { return }
        let settings: [String: Any] = [
            kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
        ]
        let out = AVAssetReaderTrackOutput(track: videoTrack, outputSettings: settings)
        out.alwaysCopiesSampleData = false
        guard preReader.canAdd(out) else { return }
        preReader.add(out)
        guard preReader.startReading() else { return }

        bridge.resetTranslational2D(withStrength: strength)
        var frameIdx: UInt = 0
        var lastPct = -1

        while !isCancelled {
            var done = false
            autoreleasepool {
                guard let sample = out.copyNextSampleBuffer() else {
                    done = true
                    return
                }
                if let pb = CMSampleBufferGetImageBuffer(sample) {
                    bridge.feedTranslational2DFrameIndex(frameIdx, pixelBuffer: pb)
                    frameIdx += 1
                    let pct = min(100, Int((Double(frameIdx) / Double(max(1, totalFrames))) * 100.0))
                    if pct != lastPct && (pct % 5 == 0 || pct == 100) {
                        lastPct = pct
                        DispatchQueue.main.async {
                            statusUpdate?("Analyzing 2D Footstep Motion: \(pct)%")
                        }
                    }
                }
            }
            if done { break }
        }
        preReader.cancelReading()
        if !isCancelled {
            bridge.finalizeTranslational2D()
            FileLogger.shared.log(String(
                format: "Translational2D (160x90): Valid=%lu/%d | MaxShift=%.1f px | SafeZoom=[%.2fx..%.2fx]",
                bridge.plan.translational2DValidFrames, totalFrames, bridge.plan.hybridResidualMaxPx,
                bridge.plan.minZoom, bridge.plan.maxZoom
            ))
        }
    }

    private func makeBridge(sidecarPath: String, config: StabilizeExportConfig, videoRotationDegrees: Int) -> GFStabilizerBridge? {
        return GFStabilizerBridge(
            sidecarPath: sidecarPath,
            pitchSmoothness: config.pitchSmoothness,
            yawSmoothness: config.yawSmoothness,
            rollSmoothness: config.rollSmoothness,
            velocityDamping: config.velocityDamping,
            timeOffsetMs: config.timeOffsetMs,
            maxZoomLimit: config.maxZoomLimit,
            horizonLockEnabled: config.horizonLockEnabled,
            horizonLockAmount: config.horizonLockAmount,
            horizonTiltOffsetDeg: config.horizonTiltOffsetDeg,
            horizonMode: config.horizonMode,
            rollingShutterCorrection: config.rollingShutterCorrection,
            dynamicZoomEnabled: config.dynamicZoomEnabled,
            axisPresetIndex: config.axisPresetIndex,
            videoRotationDegrees: videoRotationDegrees,
            hasReadoutOverride: config.hasReadoutOverride,
            signedReadoutTimeMs: config.signedReadoutTimeMs,
            focalScaleOverride: config.focalScaleOverride,
            rollGyroScaleOverride: config.rollGyroScaleOverride,
            motoActionMode: config.motoActionMode,
            edgeFillMode: config.edgeFillMode,
            fovScaleBoost: config.fovScaleBoost,
            sharpenStrength: config.sharpenStrength
        )
    }

    private func makeError(_ message: String) -> NSError {
        return NSError(
            domain: "GyroStabilizer5s.OfflineRender",
            code: -1,
            userInfo: [NSLocalizedDescriptionKey: message]
        )
    }
}
