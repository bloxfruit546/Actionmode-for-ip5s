import AVKit
import UIKit

final class StabilizeViewController: UIViewController {
    private let take: CaptureTake
    private let pipeline = OfflineRenderPipeline()
    private var isRendering = false
    private var lastExportedURL: URL?

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

    private let smoothLabel = UILabel()
    private let smoothSlider = UISlider()

    private let zoomLabel = UILabel()
    private let zoomSlider = UISlider()
    private let unlimitedZoomSwitch = UISwitch()

    private let offsetLabel = UILabel()
    private let offsetSlider = UISlider()

    private let previewInfoLabel: UILabel = {
        let l = UILabel()
        l.translatesAutoresizingMaskIntoConstraints = false
        l.font = UIFont(name: "Menlo", size: 11) ?? UIFont.systemFont(ofSize: 11)
        l.textColor = UIColor(red: 0.35, green: 0.95, blue: 0.55, alpha: 1.0)
        l.numberOfLines = 0
        l.text = "Tap Preview or Start to compute trajectory."
        return l
    }()

    private let progressView: UIProgressView = {
        let pv = UIProgressView(progressViewStyle: .default)
        pv.translatesAutoresizingMaskIntoConstraints = false
        pv.progressTintColor = UIColor(red: 0.25, green: 0.65, blue: 1.0, alpha: 1.0)
        pv.trackTintColor = UIColor(white: 0.25, alpha: 1.0)
        return pv
    }()

    private let statusLabel: UILabel = {
        let l = UILabel()
        l.translatesAutoresizingMaskIntoConstraints = false
        l.font = UIFont(name: "Menlo", size: 11) ?? UIFont.systemFont(ofSize: 11)
        l.textColor = UIColor(white: 0.85, alpha: 1.0)
        l.numberOfLines = 0
        l.text = "Ready to stabilize."
        return l
    }()

    private let previewButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("COMPUTE TRAJECTORY PREVIEW", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 13, weight: .semibold)
        btn.backgroundColor = UIColor(white: 0.22, alpha: 1.0)
        btn.layer.cornerRadius = 8
        return btn
    }()

    private let renderButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("STABILIZE & EXPORT (METAL)", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 15, weight: .bold)
        btn.backgroundColor = UIColor(red: 0.12, green: 0.48, blue: 0.95, alpha: 1.0)
        btn.layer.cornerRadius = 8
        return btn
    }()

    private let playButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("PLAY RESULT", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 13, weight: .bold)
        btn.backgroundColor = UIColor(red: 0.16, green: 0.62, blue: 0.36, alpha: 1.0)
        btn.layer.cornerRadius = 8
        btn.isEnabled = false
        btn.alpha = 0.4
        return btn
    }()

    private let savePhotosButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("SAVE TO PHOTOS", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 13, weight: .bold)
        btn.backgroundColor = UIColor(red: 0.72, green: 0.42, blue: 0.12, alpha: 1.0)
        btn.layer.cornerRadius = 8
        btn.isEnabled = false
        btn.alpha = 0.4
        return btn
    }()

    private let shareButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("SHARE .MOV", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 13, weight: .bold)
        btn.backgroundColor = UIColor(white: 0.25, alpha: 1.0)
        btn.layer.cornerRadius = 8
        btn.isEnabled = false
        btn.alpha = 0.4
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
        navigationItem.title = "Stabilize"
        view.backgroundColor = UIColor(red: 0.09, green: 0.10, blue: 0.12, alpha: 1.0)

        configureControls()
        setupLayout()
        updateSliderLabels()
        handlePreview()
    }

    private func configureControls() {
        for lbl in [smoothLabel, zoomLabel, offsetLabel] {
            lbl.translatesAutoresizingMaskIntoConstraints = false
            lbl.font = UIFont(name: "Menlo-Bold", size: 12) ?? UIFont.systemFont(ofSize: 12, weight: .bold)
            lbl.textColor = .white
        }

        smoothSlider.translatesAutoresizingMaskIntoConstraints = false
        smoothSlider.minimumValue = 0.0
        smoothSlider.maximumValue = 1.0
        smoothSlider.value = 0.65
        smoothSlider.addTarget(self, action: #selector(handleSliderChanged), for: .valueChanged)

        zoomSlider.translatesAutoresizingMaskIntoConstraints = false
        zoomSlider.minimumValue = 1.05
        zoomSlider.maximumValue = 1.60
        zoomSlider.value = 1.35
        zoomSlider.addTarget(self, action: #selector(handleSliderChanged), for: .valueChanged)

        unlimitedZoomSwitch.translatesAutoresizingMaskIntoConstraints = false
        unlimitedZoomSwitch.isOn = false
        unlimitedZoomSwitch.addTarget(self, action: #selector(handleSliderChanged), for: .valueChanged)

        offsetSlider.translatesAutoresizingMaskIntoConstraints = false
        offsetSlider.minimumValue = -50.0
        offsetSlider.maximumValue = 50.0
        offsetSlider.value = 0.0
        offsetSlider.addTarget(self, action: #selector(handleSliderChanged), for: .valueChanged)

        previewButton.addTarget(self, action: #selector(handlePreview), for: .touchUpInside)
        renderButton.addTarget(self, action: #selector(handleRenderTap), for: .touchUpInside)
        playButton.addTarget(self, action: #selector(handlePlay), for: .touchUpInside)
        savePhotosButton.addTarget(self, action: #selector(handleSavePhotos), for: .touchUpInside)
        shareButton.addTarget(self, action: #selector(handleShare), for: .touchUpInside)
    }

    private func setupLayout() {
        view.addSubview(scrollView)
        scrollView.addSubview(contentView)

        let zoomRow = UIStackView(arrangedSubviews: [zoomLabel, unlimitedZoomSwitch])
        zoomRow.translatesAutoresizingMaskIntoConstraints = false
        zoomRow.axis = .horizontal
        zoomRow.alignment = .center
        zoomRow.spacing = 8

        let stack = UIStackView(arrangedSubviews: [
            smoothLabel,
            smoothSlider,
            zoomRow,
            zoomSlider,
            offsetLabel,
            offsetSlider,
            previewButton,
            previewInfoLabel,
            progressView,
            statusLabel,
            renderButton,
            playButton,
            savePhotosButton,
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

            previewButton.heightAnchor.constraint(equalToConstant: 38),
            renderButton.heightAnchor.constraint(equalToConstant: 46),
            playButton.heightAnchor.constraint(equalToConstant: 42),
            savePhotosButton.heightAnchor.constraint(equalToConstant: 42),
            shareButton.heightAnchor.constraint(equalToConstant: 42)
        ])
    }

    private func currentConfig() -> StabilizeExportConfig {
        let maxZoom: Double = unlimitedZoomSwitch.isOn ? 0.0 : Double(zoomSlider.value)
        return StabilizeExportConfig(
            smoothness: Double(smoothSlider.value),
            velocityDamping: 1.0,
            timeOffsetMs: Double(offsetSlider.value),
            maxZoomLimit: maxZoom
        )
    }

    private func updateSliderLabels() {
        smoothLabel.text = String(format: "Smoothness: %.2f", smoothSlider.value)
        if unlimitedZoomSwitch.isOn {
            zoomLabel.text = "Max Zoom: UNLIMITED (0% Border)"
            zoomSlider.isEnabled = false
            zoomSlider.alpha = 0.4
        } else {
            zoomLabel.text = String(format: "Max Zoom Limit: %.2fx", zoomSlider.value)
            zoomSlider.isEnabled = true
            zoomSlider.alpha = 1.0
        }
        offsetLabel.text = String(format: "IMU Sync Offset: %+.1f ms", offsetSlider.value)
    }

    @objc private func handleSliderChanged() {
        updateSliderLabels()
    }

    @objc private func handlePreview() {
        let cfg = currentConfig()
        if let plan = pipeline.computePlanPreview(take: take, config: cfg), plan.valid {
            previewInfoLabel.textColor = UIColor(red: 0.35, green: 0.95, blue: 0.55, alpha: 1.0)
            previewInfoLabel.text = String(
                format: "Frames: %u (%ux%u)\nStability Gain: %.1f%%\nSafe Zoom: [%.2fx .. %.2fx] (avg %.2fx)",
                plan.frameCount, plan.width, plan.height,
                plan.stabilityImprovementPct,
                plan.minZoom, plan.maxZoom, plan.avgZoom
            )
        } else {
            previewInfoLabel.textColor = UIColor(red: 1.0, green: 0.45, blue: 0.45, alpha: 1.0)
            previewInfoLabel.text = "Failed to compute trajectory preview."
        }
    }

    @objc private func handleRenderTap() {
        if isRendering {
            pipeline.cancel()
            renderButton.isEnabled = false
            statusLabel.text = "Cancelling..."
            return
        }

        isRendering = true
        lastExportedURL = nil
        setResultButtonsEnabled(false)
        progressView.progress = 0.0
        renderButton.setTitle("CANCEL RENDER", for: .normal)
        renderButton.backgroundColor = UIColor(red: 0.85, green: 0.22, blue: 0.22, alpha: 1.0)
        statusLabel.textColor = UIColor(white: 0.90, alpha: 1.0)
        statusLabel.text = "Initializing Metal Lanczos4 pipeline..."

        let cfg = currentConfig()
        pipeline.render(
            take: take,
            config: cfg,
            progress: { [weak self] frac, frameIdx, totalFrames, ramMB in
                self?.progressView.progress = Float(frac)
                self?.statusLabel.text = String(
                    format: "Rendering %d / %d (%.0f%%) | RAM: %.1f MB",
                    frameIdx, totalFrames, frac * 100.0, ramMB
                )
            },
            completion: { [weak self] result in
                guard let self = self else { return }
                self.isRendering = false
                self.renderButton.isEnabled = true
                self.renderButton.setTitle("STABILIZE & EXPORT (METAL)", for: .normal)
                self.renderButton.backgroundColor = UIColor(red: 0.12, green: 0.48, blue: 0.95, alpha: 1.0)

                switch result {
                case .success(let res):
                    self.lastExportedURL = res.outputURL
                    self.progressView.progress = 1.0
                    self.setResultButtonsEnabled(true)
                    self.statusLabel.textColor = UIColor(red: 0.35, green: 0.95, blue: 0.55, alpha: 1.0)
                    self.statusLabel.text = String(
                        format: "DONE! %d frames in %.1fs (%.1f fps)\nPeak RAM: %.1f MB | Gain: %.1f%%",
                        res.renderedFrames, res.elapsedSeconds,
                        Double(res.renderedFrames) / max(res.elapsedSeconds, 0.01),
                        res.peakRamMB, res.plan.stabilityImprovementPct
                    )
                case .failure(let err):
                    self.statusLabel.textColor = UIColor(red: 1.0, green: 0.45, blue: 0.45, alpha: 1.0)
                    self.statusLabel.text = "Error: \(err.localizedDescription)"
                }
            }
        )
    }

    private func setResultButtonsEnabled(_ enabled: Bool) {
        for btn in [playButton, savePhotosButton, shareButton] {
            btn.isEnabled = enabled
            btn.alpha = enabled ? 1.0 : 0.4
        }
    }

    @objc private func handlePlay() {
        guard let url = lastExportedURL else { return }
        let player = AVPlayer(url: url)
        let vc = AVPlayerViewController()
        vc.player = player
        present(vc, animated: true) {
            player.play()
        }
    }

    @objc private func handleSavePhotos() {
        guard let url = lastExportedURL else { return }
        savePhotosButton.isEnabled = false
        OfflineRenderPipeline.saveVideoToPhotos(url: url) { [weak self] result in
            guard let self = self else { return }
            self.savePhotosButton.isEnabled = true
            switch result {
            case .success:
                self.statusLabel.textColor = UIColor(red: 0.35, green: 0.95, blue: 0.55, alpha: 1.0)
                self.statusLabel.text = "Saved stabilized video to Photos!"
            case .failure(let err):
                self.statusLabel.textColor = UIColor(red: 1.0, green: 0.45, blue: 0.45, alpha: 1.0)
                self.statusLabel.text = "Photos Save Error: \(err.localizedDescription)"
            }
        }
    }

    @objc private func handleShare() {
        guard let url = lastExportedURL else { return }
        let vc = UIActivityViewController(activityItems: [url], applicationActivities: nil)
        if let pop = vc.popoverPresentationController {
            pop.sourceView = shareButton
            pop.sourceRect = shareButton.bounds
        }
        present(vc, animated: true, completion: nil)
    }
}
