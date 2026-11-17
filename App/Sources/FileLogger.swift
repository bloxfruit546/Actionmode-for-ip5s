import Foundation
import UIKit

final class FileLogger {
    static let shared = FileLogger()

    private let queue = DispatchQueue(label: "com.gyrostabilizer5s.filelogger")
    private let dateFormatter: DateFormatter

    var onLogAppended: ((String) -> Void)?

    var logFileURL: URL {
        let docs = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
        return docs.appendingPathComponent("probe_logs.txt")
    }

    var documentsDirectoryPath: String {
        return logFileURL.deletingLastPathComponent().path
    }

    private init() {
        dateFormatter = DateFormatter()
        dateFormatter.locale = Locale(identifier: "en_US_POSIX")
        dateFormatter.dateFormat = "HH:mm:ss.SSS"
    }

    func log(_ message: String) {
        let timestamp = dateFormatter.string(from: Date())
        let ramMB = Double(GFSystemProbe.currentPhysFootprintBytes()) / (1024.0 * 1024.0)
        let formattedLine = String(format: "[%@] [RAM: %.1f MB] %@", timestamp, ramMB, message)

        NSLog("%@", formattedLine)

        queue.async { [weak self] in
            guard let self = self else { return }
            let data = (formattedLine + "\n").data(using: .utf8) ?? Data()
            let url = self.logFileURL

            if FileManager.default.fileExists(atPath: url.path) {
                if let handle = try? FileHandle(forWritingTo: url) {
                    handle.seekToEndOfFile()
                    handle.write(data)
                    handle.closeFile()
                }
            } else {
                try? data.write(to: url, options: .atomic)
            }

            DispatchQueue.main.async {
                self.onLogAppended?(formattedLine)
            }
        }
    }

    func readAllLogs() -> String {
        return queue.sync {
            guard let data = try? Data(contentsOf: logFileURL),
                  let text = String(data: data, encoding: .utf8) else {
                return ""
            }
            return text
        }
    }

    func clearLogs() {
        queue.sync {
            try? FileManager.default.removeItem(at: logFileURL)
        }
    }

    func presentShareSheet(from viewController: UIViewController, sourceView: UIView) {
        queue.sync {} // Ensure pending writes are flushed
        guard FileManager.default.fileExists(atPath: logFileURL.path) else {
            log("No log file found to share.")
            return
        }

        let activityVC = UIActivityViewController(activityItems: [logFileURL], applicationActivities: nil)
        if let popover = activityVC.popoverPresentationController {
            popover.sourceView = sourceView
            popover.sourceRect = sourceView.bounds
        }
        viewController.present(activityVC, animated: true, completion: nil)
    }
}
