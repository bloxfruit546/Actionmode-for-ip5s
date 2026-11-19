import AVKit
import UIKit

final class TakeDetailViewController: UIViewController {
    private let take: CaptureTake

    private let scrollView: UIScrollView = {
        let sv = UIScrollView()
        sv.translatesAutoresizingMaskIntoConstraints = false
        sv.alwaysBounceVertical = true
        return sv
    }()

    private let contentView: UIView = {
        let v = UIView()
        v.translatesAutoresizingMaskIntoConstraints = false
        return v
    }()

    private let summaryTextView: UITextView = {
        let tv = UITextView()
        tv.translatesAutoresizingMaskIntoConstraints = false
        tv.backgroundColor = UIColor(red: 0.06, green: 0.07, blue: 0.09, alpha: 1.0)
        tv.textColor = UIColor(white: 0.92, alpha: 1.0)
        tv.font = UIFont(name: "Menlo", size: 11) ?? UIFont.systemFont(ofSize: 11)
        tv.isEditable = false
        tv.layer.cornerRadius = 8
        tv.layer.borderWidth = 1
        tv.layer.borderColor = UIColor(white: 0.22, alpha: 1.0).cgColor
        return tv
    }()

    private let stabilizeButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("STABILIZE THIS TAKE", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 15, weight: .heavy)
        btn.backgroundColor = UIColor(red: 0.85, green: 0.38, blue: 0.12, alpha: 1.0)
        btn.layer.cornerRadius = 8
        return btn
    }()

    private let playButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("PLAY .MOV", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 14, weight: .bold)
        btn.backgroundColor = UIColor(red: 0.16, green: 0.62, blue: 0.36, alpha: 1.0)
        btn.layer.cornerRadius = 8
        return btn
    }()

    private let shareButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("SHARE .MOV + .GFRAW", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 14, weight: .bold)
        btn.backgroundColor = UIColor(red: 0.12, green: 0.48, blue: 0.95, alpha: 1.0)
        btn.layer.cornerRadius = 8
        return btn
    }()

    init(take: CaptureTake) {
        self.take = take
        super.init(nibName: nil, bundle: nil)
        self.hidesBottomBarWhenPushed = true
    }

    required init?(coder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }

    override func viewDidLoad() {
        super.viewDidLoad()
        navigationItem.title = String(take.uuid.prefix(8))
        view.backgroundColor = UIColor(red: 0.09, green: 0.10, blue: 0.12, alpha: 1.0)

        setupLayout()
        inspectSidecar()

        stabilizeButton.addTarget(self, action: #selector(handleStabilize), for: .touchUpInside)
        playButton.addTarget(self, action: #selector(handlePlay), for: .touchUpInside)
        shareButton.addTarget(self, action: #selector(handleShare), for: .touchUpInside)
    }

    private func setupLayout() {
        view.addSubview(scrollView)
        scrollView.addSubview(contentView)

        let stack = UIStackView(arrangedSubviews: [
            summaryTextView,
            stabilizeButton,
            playButton,
            shareButton
        ])
        stack.translatesAutoresizingMaskIntoConstraints = false
        stack.axis = .vertical
        stack.spacing = 10
        contentView.addSubview(stack)

        let guide = view.safeAreaLayoutGuide
        NSLayoutConstraint.activate([
            scrollView.topAnchor.constraint(equalTo: guide.topAnchor),
            scrollView.leadingAnchor.constraint(equalTo: guide.leadingAnchor),
            scrollView.trailingAnchor.constraint(equalTo: guide.trailingAnchor),
            scrollView.bottomAnchor.constraint(equalTo: guide.bottomAnchor),

            contentView.topAnchor.constraint(equalTo: scrollView.contentLayoutGuide.topAnchor),
            contentView.leadingAnchor.constraint(equalTo: scrollView.contentLayoutGuide.leadingAnchor),
            contentView.trailingAnchor.constraint(equalTo: scrollView.contentLayoutGuide.trailingAnchor),
            contentView.bottomAnchor.constraint(equalTo: scrollView.contentLayoutGuide.bottomAnchor),
            contentView.widthAnchor.constraint(equalTo: scrollView.frameLayoutGuide.widthAnchor),

            stack.topAnchor.constraint(equalTo: contentView.topAnchor, constant: 12),
            stack.leadingAnchor.constraint(equalTo: contentView.leadingAnchor, constant: 12),
            stack.trailingAnchor.constraint(equalTo: contentView.trailingAnchor, constant: -12),
            stack.bottomAnchor.constraint(equalTo: contentView.bottomAnchor, constant: -28),

            summaryTextView.heightAnchor.constraint(equalToConstant: 265),
            stabilizeButton.heightAnchor.constraint(equalToConstant: 46),
            playButton.heightAnchor.constraint(equalToConstant: 42),
            shareButton.heightAnchor.constraint(equalToConstant: 42)
        ])
    }

    private func inspectSidecar() {
        let s = GFSidecarReaderBridge.inspectFile(atPath: take.sidecarURL.path)
        let movMB = Double(take.movSizeBytes) / (1024.0 * 1024.0)
        let gfKB = Double(take.sidecarSizeBytes) / 1024.0
        let sourceBadge = s.isVirtualGyro
            ? "VIRTUAL GYRO (OPTICAL RANSAC)\n        Note: Horizon Lock is Best-Effort (no real Accel)."
            : "HARDWARE IMU (90Hz MEMS)"

        let minShutterDenom = (s.minExposureMs > 0.01) ? (1000.0 / s.minExposureMs) : 0.0
        let maxShutterDenom = (s.maxExposureMs > 0.01) ? (1000.0 / s.maxExposureMs) : 0.0

        summaryTextView.text = String(format: """
        Take UUID     : %@
        Source        : %@
        .mov Size     : %.2f MB
        .gfraw Size   : %.1f KB
        ----------------------------------------
        Header Valid  : %@
        Footer Clean  : %@
        Resolution    : %ux%u (mode=%u, nom=%.1f fps)
        Duration      : %.2f s
        Readout Time  : %.2f ms
        Gyro Bias     : (%+.5f, %+.5f, %+.5f)
        ----------------------------------------
        Gyro Samples  : %llu (%.2f Hz)
        Accel Samples : %llu (%.2f Hz)
        Video Frames  : %llu (avg %.2f fps)
        Dropped Frames: %u
        Exposure Range: %.2f .. %.2f ms (1/%.0f .. 1/%.0f s)
        Sync Coverage : %@
        Error         : %@
        """,
        take.uuid,
        sourceBadge,
        movMB,
        gfKB,
        s.validHeader ? "YES (GFS5 v2)" : "NO",
        s.hasFooter ? "YES" : "NO (Recovered to EOF)",
        s.width, s.height, s.captureMode, s.nominalFps,
        s.recordingDurationSec,
        s.readoutTimeSec * 1000.0,
        s.gyroBiasX, s.gyroBiasY, s.gyroBiasZ,
        s.gyroCount, s.gyroHz,
        s.accelCount, s.accelHz,
        s.frameCount, s.avgFps,
        s.droppedFrames,
        s.minExposureMs, s.maxExposureMs, minShutterDenom, maxShutterDenom,
        s.syncCoverageOk ? "PASS" : "WARN",
        s.errorMessage.isEmpty ? "None" : s.errorMessage)
    }

    @objc private func handleStabilize() {
        let vc = StabilizeViewController(take: take)
        navigationController?.pushViewController(vc, animated: true)
    }

    @objc private func handlePlay() {
        let player = AVPlayer(url: take.movURL)
        let playerVC = AVPlayerViewController()
        playerVC.player = player
        present(playerVC, animated: true) {
            player.play()
        }
    }

    @objc private func handleShare() {
        var items: [URL] = [take.movURL]
        if FileManager.default.fileExists(atPath: take.sidecarURL.path) {
            items.append(take.sidecarURL)
        }
        let activityVC = UIActivityViewController(activityItems: items, applicationActivities: nil)
        if let popover = activityVC.popoverPresentationController {
            popover.sourceView = shareButton
            popover.sourceRect = shareButton.bounds
        }
        present(activityVC, animated: true, completion: nil)
    }
}
