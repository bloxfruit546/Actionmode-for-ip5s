import AVFoundation
import CoreVideo
import Foundation
import Photos

struct StabilizeExportConfig {
    let smoothness: Double
    let velocityDamping: Double
    let timeOffsetMs: Double
    let maxZoomLimit: Double // <= 1.0 means unlimited safe zoom
}

struct StabilizeExportResult {
    let outputURL: URL
    let plan: GFStabilizerPlanObjC
    let renderedFrames: Int
    let elapsedSeconds: Double
    let peakRamMB: Double
}

final class OfflineRenderPipeline {
    private let renderQueue = DispatchQueue(label: "com.gyrostabilizer5s.offlinerender", qos: .userInitiated)
    private var isCancelled = false

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

    func computePlanPreview(take: CaptureTake, config: StabilizeExportConfig) -> GFStabilizerPlanObjC? {
        let bridge = GFStabilizerBridge(
            sidecarPath: take.sidecarURL.path,
            smoothness: config.smoothness,
            velocityDamping: config.velocityDamping,
            timeOffsetMs: config.timeOffsetMs,
            maxZoomLimit: config.maxZoomLimit
        )
        return bridge?.plan
    }

    func render(
        take: CaptureTake,
        config: StabilizeExportConfig,
        progress: @escaping (_ fraction: Double, _ frameIndex: Int, _ totalFrames: Int, _ ramMB: Double) -> Void,
        completion: @escaping (Result<StabilizeExportResult, Error>) -> Void
    ) {
        isCancelled = false
        renderQueue.async { [weak self] in
            guard let self = self else { return }
            let startTime = CFAbsoluteTimeGetCurrent()

            guard let bridge = GFStabilizerBridge(
                sidecarPath: take.sidecarURL.path,
                smoothness: config.smoothness,
                velocityDamping: config.velocityDamping,
                timeOffsetMs: config.timeOffsetMs,
                maxZoomLimit: config.maxZoomLimit
            ) else {
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

            let outURL = Self.stabilizedDirectory.appendingPathComponent("\(take.uuid)_stab.mov")
            try? FileManager.default.removeItem(at: outURL)

            let asset = AVURLAsset(url: take.movURL)
            guard let videoTrack = asset.tracks(withMediaType: .video).first else {
                DispatchQueue.main.async {
                    completion(.failure(self.makeError("No video track found in .mov.")))
                }
                return
            }

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
                let bitrate = (height >= 1000) ? 45_000_000 : 35_000_000
                let compressionProps: [String: Any] = [
                    AVVideoAverageBitRateKey: bitrate,
                    AVVideoProfileLevelKey: AVVideoProfileLevelH264HighAutoLevel,
                    AVVideoExpectedSourceFrameRateKey: Int(plan.nominalFps > 1 ? plan.nominalFps : 30)
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

                guard reader.startReading(), writer.startWriting() else {
                    throw self.makeError("Failed to start AVAssetReader/Writer.")
                }
                writer.startSession(atSourceTime: .zero)

                var frameIndex = 0
                var peakRamMB = Double(GFSystemProbe.currentPhysFootprintBytes()) / (1024.0 * 1024.0)

                // Render video frames sequentially inside autoreleasepool
                while !self.isCancelled {
                    var didFinishVideo = false
                    autoreleasepool {
                        guard let sample = videoReaderOut.copyNextSampleBuffer() else {
                            didFinishVideo = true
                            return
                        }
                        guard let srcPB = CMSampleBufferGetImageBuffer(sample) else {
                            return
                        }
                        let pts = CMSampleBufferGetPresentationTimeStamp(sample)

                        while !videoWriterIn.isReadyForMoreMediaData && !self.isCancelled {
                            Thread.sleep(forTimeInterval: 0.002)
                        }
                        if self.isCancelled {
                            return
                        }

                        guard let pool = adaptor.pixelBufferPool else {
                            return
                        }
                        var dstPBOpt: CVPixelBuffer?
                        let status = CVPixelBufferPoolCreatePixelBuffer(kCFAllocatorDefault, pool, &dstPBOpt)
                        guard status == kCVReturnSuccess, let dstPB = dstPBOpt else {
                            return
                        }

                        _ = bridge.warpPixelBuffer(srcPB, intoPixelBuffer: dstPB, frameIndex: UInt(frameIndex))
                        _ = adaptor.append(dstPB, withPresentationTime: pts)

                        frameIndex += 1
                        if frameIndex % 30 == 0 {
                            bridge.flushTextureCache()
                        }

                        let ramMB = Double(GFSystemProbe.currentPhysFootprintBytes()) / (1024.0 * 1024.0)
                        if ramMB > peakRamMB {
                            peakRamMB = ramMB
                        }
                        if frameIndex % 5 == 0 || frameIndex == totalFrames {
                            let frac = min(1.0, Double(frameIndex) / Double(totalFrames))
                            let curIdx = frameIndex
                            DispatchQueue.main.async {
                                progress(frac, curIdx, totalFrames, ramMB)
                            }
                        }
                    }
                    if didFinishVideo {
                        break
                    }
                }

                if self.isCancelled {
                    reader.cancelReading()
                    writer.cancelWriting()
                    try? FileManager.default.removeItem(at: outURL)
                    DispatchQueue.main.async {
                        completion(.failure(self.makeError("Render cancelled by user.")))
                    }
                    return
                }

                videoWriterIn.markAsFinished()

                // Audio passthrough
                if let aOut = audioReaderOut, let aIn = audioWriterIn {
                    while !self.isCancelled {
                        var didFinishAudio = false
                        autoreleasepool {
                            guard let aSample = aOut.copyNextSampleBuffer() else {
                                didFinishAudio = true
                                return
                            }
                            while !aIn.isReadyForMoreMediaData && !self.isCancelled {
                                Thread.sleep(forTimeInterval: 0.002)
                            }
                            if !self.isCancelled {
                                aIn.append(aSample)
                            }
                        }
                        if didFinishAudio {
                            break
                        }
                    }
                    aIn.markAsFinished()
                }

                let sem = DispatchSemaphore(value: 0)
                writer.finishWriting {
                    sem.signal()
                }
                sem.wait()

                bridge.flushTextureCache()

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
                        format: "Stabilize Export DONE: %d frames in %.2fs (%.1f fps) | PeakRAM=%.1f MB | Gain=%.1f%%",
                        frameIndex, elapsed, Double(frameIndex) / max(elapsed, 0.01),
                        peakRamMB, plan.stabilityImprovementPct
                    ))
                    DispatchQueue.main.async {
                        completion(.success(res))
                    }
                } else {
                    throw self.makeError(writer.error?.localizedDescription ?? "AVAssetWriter failed.")
                }
            } catch {
                try? FileManager.default.removeItem(at: outURL)
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

    private func makeError(_ message: String) -> NSError {
        return NSError(
            domain: "GyroStabilizer5s.OfflineRender",
            code: -1,
            userInfo: [NSLocalizedDescriptionKey: message]
        )
    }
}
