import Foundation
import Photos
import UIKit

final class PhotosSaver: NSObject {
    static let shared = PhotosSaver()

    private var legacySaveCompletion: ((Bool, Error?) -> Void)?

    func saveVideoToPhotos(
        videoURL: URL,
        from viewController: UIViewController?,
        completion: @escaping (Result<Void, Error>) -> Void
    ) {
        FileLogger.shared.log("PhotosSaver: Starting save for \(videoURL.lastPathComponent)")

        // Step 1: File existence and non-zero size validation
        guard FileManager.default.fileExists(atPath: videoURL.path) else {
            let err = NSError(domain: "PhotosSaver", code: -1, userInfo: [NSLocalizedDescriptionKey: "Rendered video file not found at \(videoURL.path)"])
            FileLogger.shared.log("PhotosSaver ERROR: \(err.localizedDescription)")
            DispatchQueue.main.async { completion(.failure(err)) }
            return
        }

        let fileSize = (try? FileManager.default.attributesOfItem(atPath: videoURL.path)[.size] as? Int64) ?? 0
        guard fileSize > 0 else {
            let err = NSError(domain: "PhotosSaver", code: -2, userInfo: [NSLocalizedDescriptionKey: "Rendered video file is empty (0 bytes)"])
            FileLogger.shared.log("PhotosSaver ERROR: \(err.localizedDescription)")
            DispatchQueue.main.async { completion(.failure(err)) }
            return
        }
        FileLogger.shared.log(String(format: "PhotosSaver: Video size = %.2f MB", Double(fileSize) / (1024.0 * 1024.0)))

        // Step 2: iOS Camera Roll compatibility diagnostic check
        let isCompatible = UIVideoAtPathIsCompatibleWithSavedPhotosAlbum(videoURL.path)
        FileLogger.shared.log("PhotosSaver: UIVideoAtPathIsCompatibleWithSavedPhotosAlbum = \(isCompatible)")

        // Step 3: Authorization status checking on iOS 12
        let status = PHPhotoLibrary.authorizationStatus()
        FileLogger.shared.log("PhotosSaver: PHPhotoLibrary.authorizationStatus = \(status.rawValue)")

        switch status {
        case .authorized:
            performTier1Save(videoURL: videoURL, from: viewController, completion: completion)

        case .notDetermined:
            FileLogger.shared.log("PhotosSaver: Requesting photo library authorization...")
            PHPhotoLibrary.requestAuthorization { newStatus in
                DispatchQueue.main.async {
                    if newStatus == .authorized {
                        FileLogger.shared.log("PhotosSaver: User granted Photos permission.")
                        self.performTier1Save(videoURL: videoURL, from: viewController, completion: completion)
                    } else {
                        let err = NSError(domain: "PhotosSaver", code: -3, userInfo: [NSLocalizedDescriptionKey: "Photos access was not granted."])
                        FileLogger.shared.log("PhotosSaver: Authorization rejected: \(newStatus.rawValue)")
                        self.showPermissionDeniedAlert(from: viewController)
                        completion(.failure(err))
                    }
                }
            }

        case .denied, .restricted:
            FileLogger.shared.log("PhotosSaver: Permission denied or restricted.")
            DispatchQueue.main.async {
                self.showPermissionDeniedAlert(from: viewController)
                let err = NSError(domain: "PhotosSaver", code: -4, userInfo: [NSLocalizedDescriptionKey: "Photo Library permission is denied or restricted. Please enable Photos access in Settings."])
                completion(.failure(err))
            }

        @unknown default:
            performTier1Save(videoURL: videoURL, from: viewController, completion: completion)
        }
    }

    private func showPermissionDeniedAlert(from viewController: UIViewController?) {
        guard let vc = viewController else { return }
        let alert = UIAlertController(
            title: "Photos Access Required",
            message: "GyroStabilizer5s needs permission to save stabilized videos to your Photo Library. Please tap Settings and enable Photos access.",
            preferredStyle: .alert
        )
        if let settingsURL = URL(string: UIApplication.openSettingsURLString),
           UIApplication.shared.canOpenURL(settingsURL) {
            alert.addAction(UIAlertAction(title: "Open Settings", style: .default) { _ in
                UIApplication.shared.open(settingsURL, options: [:], completionHandler: nil)
            })
        }
        alert.addAction(UIAlertAction(title: "Cancel", style: .cancel, handler: nil))
        vc.present(alert, animated: true, completion: nil)
    }

    // MARK: - Tier 1: Direct PHPhotoLibrary performChanges
    private func performTier1Save(
        videoURL: URL,
        from viewController: UIViewController?,
        completion: @escaping (Result<Void, Error>) -> Void
    ) {
        FileLogger.shared.log("PhotosSaver: Attempting Tier 1 (PHPhotoLibrary.performChanges)...")
        PHPhotoLibrary.shared().performChanges({
            PHAssetChangeRequest.creationRequestForAssetFromVideo(atFileURL: videoURL)
        }) { success, error in
            DispatchQueue.main.async {
                if success {
                    FileLogger.shared.log("PhotosSaver: Tier 1 SAVE SUCCESSFUL via PHPhotoLibrary!")
                    completion(.success(()))
                } else {
                    let errDesc = error?.localizedDescription ?? "Unknown PHPhotoLibrary error"
                    FileLogger.shared.log("PhotosSaver: Tier 1 FAILED (\(errDesc)). Falling back to Tier 2 (UISaveVideoAtPathToSavedPhotosAlbum)...")
                    self.performTier2Fallback(videoURL: videoURL, from: viewController, tier1Error: error, completion: completion)
                }
            }
        }
    }

    // MARK: - Tier 2: UISaveVideoAtPathToSavedPhotosAlbum with clean temporary file
    private func performTier2Fallback(
        videoURL: URL,
        from viewController: UIViewController?,
        tier1Error: Error?,
        completion: @escaping (Result<Void, Error>) -> Void
    ) {
        let tempDir = NSTemporaryDirectory()
        let cleanTempURL = URL(fileURLWithPath: tempDir).appendingPathComponent("EXPORT_\(UUID().uuidString.prefix(8)).MOV")
        try? FileManager.default.removeItem(at: cleanTempURL)

        do {
            try FileManager.default.copyItem(at: videoURL, to: cleanTempURL)
            FileLogger.shared.log("PhotosSaver: Copied to clean temp path \(cleanTempURL.path)")
        } catch {
            FileLogger.shared.log("PhotosSaver: Copy failed (\(error.localizedDescription)), using original path")
        }

        let pathToSave = FileManager.default.fileExists(atPath: cleanTempURL.path) ? cleanTempURL.path : videoURL.path

        self.legacySaveCompletion = { [weak self] success, legacyError in
            try? FileManager.default.removeItem(at: cleanTempURL)
            DispatchQueue.main.async {
                if success {
                    FileLogger.shared.log("PhotosSaver: Tier 2 SAVE SUCCESSFUL via UISaveVideoAtPathToSavedPhotosAlbum!")
                    completion(.success(()))
                } else {
                    let errDesc = legacyError?.localizedDescription ?? (tier1Error?.localizedDescription ?? "Unknown save error")
                    FileLogger.shared.log("PhotosSaver: Tier 2 FAILED (\(errDesc)). Offering Tier 3 (Share Sheet)...")
                    self?.performTier3Insurance(videoURL: videoURL, from: viewController, errorDescription: errDesc, completion: completion)
                }
            }
        }

        UISaveVideoAtPathToSavedPhotosAlbum(
            pathToSave,
            self,
            #selector(video(_:didFinishSavingWithError:contextInfo:)),
            nil
        )
    }

    @objc private func video(_ videoPath: String, didFinishSavingWithError error: Error?, contextInfo: UnsafeMutableRawPointer?) {
        let success = (error == nil)
        legacySaveCompletion?(success, error)
        legacySaveCompletion = nil
    }

    // MARK: - Tier 3: SpringBoard System Share Sheet Insurance
    private func performTier3Insurance(
        videoURL: URL,
        from viewController: UIViewController?,
        errorDescription: String,
        completion: @escaping (Result<Void, Error>) -> Void
    ) {
        let finalError = NSError(domain: "PhotosSaver", code: -5, userInfo: [NSLocalizedDescriptionKey: errorDescription])
        guard let vc = viewController else {
            completion(.failure(finalError))
            return
        }

        let alert = UIAlertController(
            title: "Photos Library Notice",
            message: "Direct save was blocked by iOS sandbox (\(errorDescription)). You can save directly to Photos using the native iOS Share Sheet below:",
            preferredStyle: .alert
        )
        alert.addAction(UIAlertAction(title: "Save via iOS Share Sheet", style: .default) { _ in
            let activityVC = UIActivityViewController(activityItems: [videoURL], applicationActivities: nil)
            if let pop = activityVC.popoverPresentationController {
                pop.sourceView = vc.view
                pop.sourceRect = CGRect(x: vc.view.bounds.midX, y: vc.view.bounds.midY, width: 0, height: 0)
            }
            activityVC.completionWithItemsHandler = { activityType, completed, returnedItems, activityError in
                if completed {
                    FileLogger.shared.log("PhotosSaver: Saved via Share Sheet!")
                    completion(.success(()))
                } else if let actErr = activityError {
                    completion(.failure(actErr))
                }
            }
            vc.present(activityVC, animated: true, completion: nil)
        })
        alert.addAction(UIAlertAction(title: "Dismiss", style: .cancel) { _ in
            completion(.failure(finalError))
        })
        vc.present(alert, animated: true, completion: nil)
    }
}
