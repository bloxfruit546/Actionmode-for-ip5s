import Foundation
import UIKit

enum RenderJobStatus: String, Codable {
    case pending
    case running
    case paused
    case completed
    case failed
}

struct RenderQueueJob: Codable {
    let jobId: String
    let takeUUID: String
    let movFileName: String
    let sidecarFileName: String
    let config: StabilizeExportConfig
    var status: RenderJobStatus
    var progress: Double
    var completedFrames: Int
    var totalFrames: Int
    var outputFileName: String?
    var errorMessage: String?
    let createdAt: Date
}

final class RenderQueueManager {
    static let shared = RenderQueueManager()
    static let didUpdateNotification = Notification.Name("RenderQueueManagerDidUpdate")

    private(set) var jobs: [RenderQueueJob] = []
    private let pipeline = OfflineRenderPipeline()
    private var isProcessing = false
    private var activeJobId: String?

    private var queueFileURL: URL {
        let docs = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
        let dir = docs.appendingPathComponent("Queue", isDirectory: true)
        if !FileManager.default.fileExists(atPath: dir.path) {
            try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true, attributes: nil)
        }
        return dir.appendingPathComponent("render_queue.json")
    }

    private init() {
        loadFromDisk()
        NotificationCenter.default.addObserver(
            self,
            selector: #selector(handleAppWillResignActive),
            name: UIApplication.willResignActiveNotification,
            object: nil
        )
        NotificationCenter.default.addObserver(
            self,
            selector: #selector(handleMemoryWarning),
            name: UIApplication.didReceiveMemoryWarningNotification,
            object: nil
        )
    }

    func enqueue(take: CaptureTake, config: StabilizeExportConfig) {
        let job = RenderQueueJob(
            jobId: UUID().uuidString,
            takeUUID: take.uuid,
            movFileName: take.movURL.lastPathComponent,
            sidecarFileName: take.sidecarURL.lastPathComponent,
            config: config,
            status: .pending,
            progress: 0.0,
            completedFrames: 0,
            totalFrames: 0,
            outputFileName: nil,
            errorMessage: nil,
            createdAt: Date()
        )
        jobs.append(job)
        saveToDisk()
        postUpdate()
        processNextIfNeeded()
    }

    func resumePendingJobs() {
        var changed = false
        for i in jobs.indices {
            if jobs[i].status == .paused || jobs[i].status == .running {
                jobs[i].status = .pending
                changed = true
            }
        }
        if changed {
            saveToDisk()
            postUpdate()
        }
        processNextIfNeeded()
    }

    func removeJob(at index: Int) {
        guard index >= 0 && index < jobs.count else { return }
        if jobs[index].jobId == activeJobId {
            pipeline.cancel()
        }
        jobs.remove(at: index)
        saveToDisk()
        postUpdate()
    }

    func clearCompleted() {
        jobs.removeAll { $0.status == .completed || $0.status == .failed }
        saveToDisk()
        postUpdate()
    }

    @objc private func handleAppWillResignActive() {
        pauseActiveJobCheckpoint()
    }

    @objc private func handleMemoryWarning() {
        pauseActiveJobCheckpoint()
    }

    private func pauseActiveJobCheckpoint() {
        if let activeId = activeJobId, let idx = jobs.firstIndex(where: { $0.jobId == activeId }) {
            jobs[idx].status = .paused
            pipeline.cancel()
            isProcessing = false
            activeJobId = nil
        }
        saveToDisk()
        postUpdate()
    }

    private func processNextIfNeeded() {
        guard !isProcessing else { return }
        guard let idx = jobs.firstIndex(where: { $0.status == .pending }) else { return }

        isProcessing = true
        let job = jobs[idx]
        activeJobId = job.jobId
        jobs[idx].status = .running
        jobs[idx].errorMessage = nil
        saveToDisk()
        postUpdate()

        let capturesDir = CaptureTakeStore.capturesDirectory
        let movURL = capturesDir.appendingPathComponent(job.movFileName)
        let sidecarURL = capturesDir.appendingPathComponent(job.sidecarFileName)
        let take = CaptureTake(
            uuid: job.takeUUID,
            movURL: movURL,
            sidecarURL: sidecarURL,
            createdAt: job.createdAt,
            movSizeBytes: 0,
            sidecarSizeBytes: 0
        )

        pipeline.render(
            take: take,
            config: job.config,
            progress: { [weak self] frac, frameIdx, totalFrames, _ in
                guard let self = self,
                      let jIdx = self.jobs.firstIndex(where: { $0.jobId == job.jobId }) else { return }
                self.jobs[jIdx].progress = frac
                self.jobs[jIdx].completedFrames = frameIdx
                self.jobs[jIdx].totalFrames = totalFrames
                if frameIdx % 15 == 0 {
                    self.saveToDisk()
                }
                self.postUpdate()
            },
            completion: { [weak self] result in
                guard let self = self else { return }
                self.isProcessing = false
                self.activeJobId = nil
                guard let jIdx = self.jobs.firstIndex(where: { $0.jobId == job.jobId }) else {
                    self.processNextIfNeeded()
                    return
                }
                switch result {
                case .success(let res):
                    self.jobs[jIdx].status = .completed
                    self.jobs[jIdx].progress = 1.0
                    self.jobs[jIdx].completedFrames = res.renderedFrames
                    self.jobs[jIdx].totalFrames = res.renderedFrames
                    self.jobs[jIdx].outputFileName = res.outputURL.lastPathComponent
                    OfflineRenderPipeline.saveVideoToPhotos(url: res.outputURL) { _ in }
                case .failure(let err):
                    if self.jobs[jIdx].status != .paused {
                        self.jobs[jIdx].status = .failed
                        self.jobs[jIdx].errorMessage = err.localizedDescription
                    }
                }
                self.saveToDisk()
                self.postUpdate()
                self.processNextIfNeeded()
            }
        )
    }

    private func loadFromDisk() {
        guard let data = try? Data(contentsOf: queueFileURL),
              var decoded = try? JSONDecoder().decode([RenderQueueJob].self, from: data) else {
            return
        }
        // Any job left "running" when app terminated is marked "paused" for safe recovery
        for i in decoded.indices where decoded[i].status == .running {
            decoded[i].status = .paused
        }
        jobs = decoded
        saveToDisk()
    }

    private func saveToDisk() {
        guard let data = try? JSONEncoder().encode(jobs) else { return }
        try? data.write(to: queueFileURL, options: .atomic)
    }

    private func postUpdate() {
        DispatchQueue.main.async {
            NotificationCenter.default.post(name: RenderQueueManager.didUpdateNotification, object: nil)
        }
    }
}
