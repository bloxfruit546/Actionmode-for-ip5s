import AVFoundation
import CoreVideo
import Foundation
import Photos
import UIKit

final class PhotosImporter {
    enum ImportError: LocalizedError {
        case missingMediaURL
        case noVideoTrack
        case readerSetupFailed(String)
        case virtualGyroFailed(String)

        var errorDescription: String? {
            switch self {
            case .missingMediaURL:
                return "No video URL returned from Photos."
            case .noVideoTrack:
                return "Selected media does not contain a video track."
            case .readerSetupFailed(let msg):
                return "AVAssetReader failed: \(msg)"
            case .virtualGyroFailed(let msg):
                return "Virtual Gyro generation failed: \(msg)"
            }
        }
    }

    static func requestPhotoPermissionIfNeeded(completion: @escaping (Bool) -> Void) {
        let status = PHPhotoLibrary.authorizationStatus()
        switch status {
        case .authorized:
            completion(true)
        case .notDetermined:
            PHPhotoLibrary.requestAuthorization { newStatus in
                DispatchQueue.main.async {
                    completion(newStatus == .authorized)
                }
            }
        default:
            completion(false)
        }
    }

    static func importOriginalVideo(
        fromPickerInfo info: [UIImagePickerController.InfoKey: Any],
        progress: @escaping (Double, String) -> Void,
        completion: @escaping (Result<CaptureTake, Error>) -> Void
    ) {
        let paths = CaptureTakeStore.makeNewTakePaths()

        if let phAsset = info[.phAsset] as? PHAsset {
            let resources = PHAssetResource.assetResources(for: phAsset)
            if let videoRes = resources.first(where: { $0.type == .video }) ?? resources.first {
                DispatchQueue.main.async {
                    progress(0.05, "Copying original video from Photos...")
                }
                let opts = PHAssetResourceRequestOptions()
                opts.isNetworkAccessAllowed = true
                PHAssetResourceManager.default().writeData(for: videoRes, toFile: paths.movURL, options: opts) { err in
                    if let err = err {
                        // Fallback to mediaURL if PHAssetResourceManager fails
                        self.fallbackCopyMediaURL(info: info, paths: paths, progress: progress, completion: completion, priorError: err)
                    } else {
                        self.generateVirtualGyro(paths: paths, progress: progress, completion: completion)
                    }
                }
                return
            }
        }

        fallbackCopyMediaURL(info: info, paths: paths, progress: progress, completion: completion, priorError: nil)
    }

    private static func fallbackCopyMediaURL(
        info: [UIImagePickerController.InfoKey: Any],
        paths: (uuid: String, movURL: URL, sidecarURL: URL),
        progress: @escaping (Double, String) -> Void,
        completion: @escaping (Result<CaptureTake, Error>) -> Void,
        priorError: Error?
    ) {
        guard let mediaURL = info[.mediaURL] as? URL else {
            DispatchQueue.main.async {
                completion(.failure(priorError ?? ImportError.missingMediaURL))
            }
            return
        }

        DispatchQueue.global(qos: .userInitiated).async {
            do {
                DispatchQueue.main.async {
                    progress(0.08, "Copying video file...")
                }
                if FileManager.default.fileExists(atPath: paths.movURL.path) {
                    try FileManager.default.removeItem(at: paths.movURL)
                }
                try FileManager.default.copyItem(at: mediaURL, to: paths.movURL)
                generateVirtualGyro(paths: paths, progress: progress, completion: completion)
            } catch {
                DispatchQueue.main.async {
                    completion(.failure(error))
                }
            }
        }
    }

    private static func generateVirtualGyro(
        paths: (uuid: String, movURL: URL, sidecarURL: URL),
        progress: @escaping (Double, String) -> Void,
        completion: @escaping (Result<CaptureTake, Error>) -> Void
    ) {
        DispatchQueue.global(qos: .userInitiated).async {
            let asset = AVURLAsset(url: paths.movURL)
            guard let videoTrack = asset.tracks(withMediaType: .video).first else {
                try? FileManager.default.removeItem(at: paths.movURL)
                DispatchQueue.main.async { completion(.failure(ImportError.noVideoTrack)) }
                return
            }

            let natSize = videoTrack.naturalSize
            let width = UInt32(max(1.0, abs(natSize.width)))
            let height = UInt32(max(1.0, abs(natSize.height)))
            let fps = Double(videoTrack.nominalFrameRate > 1.0 ? videoTrack.nominalFrameRate : 30.0)
            let durationSec = max(0.1, CMTimeGetSeconds(asset.duration))
            let readoutSec = (fps > 60.0) ? 0.0080 : 0.0152
            let fx = Double(width) * (1815.0 / 1920.0)
            let fy = fx
            let cx = Double(width) * 0.5
            let cy = Double(height) * 0.5

            let builder = GFVirtualGyroBuilderBridge(
                width: width,
                height: height,
                nominalFps: fps,
                readoutTimeSec: readoutSec,
                fx: fx,
                fy: fy,
                cx: cx,
                cy: cy,
                uuid: paths.uuid
            )

            let reader: AVAssetReader
            do {
                reader = try AVAssetReader(asset: asset)
            } catch {
                try? FileManager.default.removeItem(at: paths.movURL)
                DispatchQueue.main.async { completion(.failure(ImportError.readerSetupFailed(error.localizedDescription))) }
                return
            }

            let outputSettings: [String: Any] = [
                kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
            ]
            let trackOutput = AVAssetReaderTrackOutput(track: videoTrack, outputSettings: outputSettings)
            trackOutput.alwaysCopiesSampleData = false
            guard reader.canAdd(trackOutput) else {
                try? FileManager.default.removeItem(at: paths.movURL)
                DispatchQueue.main.async { completion(.failure(ImportError.readerSetupFailed("Cannot add track output"))) }
                return
            }
            reader.add(trackOutput)

            guard reader.startReading() else {
                try? FileManager.default.removeItem(at: paths.movURL)
                let msg = reader.error?.localizedDescription ?? "startReading failed"
                DispatchQueue.main.async { completion(.failure(ImportError.readerSetupFailed(msg))) }
                return
            }

            var frameCount = 0
            while let sampleBuffer = trackOutput.copyNextSampleBuffer() {
                autoreleasepool {
                    if let pb = CMSampleBufferGetImageBuffer(sampleBuffer) {
                        let pts = CMTimeGetSeconds(CMSampleBufferGetPresentationTimeStamp(sampleBuffer))
                        CVPixelBufferLockBaseAddress(pb, .readOnly)
                        let srcW = CVPixelBufferGetWidthOfPlane(pb, 0)
                        let srcH = CVPixelBufferGetHeightOfPlane(pb, 0)
                        let stride = CVPixelBufferGetBytesPerRowOfPlane(pb, 0)
                        if let basePtr = CVPixelBufferGetBaseAddressOfPlane(pb, 0)?.assumingMemoryBound(to: UInt8.self),
                           srcW >= 64, srcH >= 64 {
                            let targetW = min(480, srcW)
                            let step = max(1, srcW / targetW)
                            let dstW = srcW / step
                            let dstH = srcH / step
                            var luma = Data(count: dstW * dstH)
                            luma.withUnsafeMutableBytes { rawBuf in
                                guard let dst = rawBuf.baseAddress?.assumingMemoryBound(to: UInt8.self) else { return }
                                for y in 0..<dstH {
                                    let srcRow = basePtr.advanced(by: y * step * stride)
                                    let dstRow = dst.advanced(by: y * dstW)
                                    for x in 0..<dstW {
                                        dstRow[x] = srcRow[x * step]
                                    }
                                }
                            }
                            _ = builder.pushLumaData(luma, width: Int32(dstW), height: Int32(dstH), timestampSec: pts)
                        }
                        CVPixelBufferUnlockBaseAddress(pb, .readOnly)

                        frameCount += 1
                        if frameCount % 15 == 0 {
                            let frac = min(0.95, 0.10 + 0.85 * (pts / durationSec))
                            DispatchQueue.main.async {
                                progress(frac, "Generating Virtual Gyro (RANSAC)... \(frameCount) frames")
                            }
                        }
                    }
                }
            }

            DispatchQueue.main.async {
                progress(0.97, "Writing Virtual Gyro sidecar...")
            }
            let res = builder.finalizeAndWrite(toPath: paths.sidecarURL.path)
            if !res.success {
                try? FileManager.default.removeItem(at: paths.movURL)
                try? FileManager.default.removeItem(at: paths.sidecarURL)
                DispatchQueue.main.async {
                    completion(.failure(ImportError.virtualGyroFailed(res.errorMessage)))
                }
                return
            }

            let movAttrs = try? FileManager.default.attributesOfItem(atPath: paths.movURL.path)
            let scAttrs = try? FileManager.default.attributesOfItem(atPath: paths.sidecarURL.path)
            let take = CaptureTake(
                uuid: paths.uuid,
                movURL: paths.movURL,
                sidecarURL: paths.sidecarURL,
                createdAt: Date(),
                movSizeBytes: (movAttrs?[.size] as? NSNumber)?.uint64Value ?? 0,
                sidecarSizeBytes: (scAttrs?[.size] as? NSNumber)?.uint64Value ?? 0
            )
            DispatchQueue.main.async {
                progress(1.0, "Done")
                completion(.success(take))
            }
        }
    }
}
