import AVKit
import UIKit

final class StabilizeViewController: UIViewController {
    private let take: CaptureTake
    private let pipeline = OfflineRenderPipeline()
    private var isRendering = false
    private var lastExportedURL: URL?
    private var detectedAxisPresetIndex: Int = -1

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

    private let autoSyncButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("AUTO-SYNC & VERIFY AXES", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 13, weight: .bold)
        btn.backgroundColor = UIColor(red: 0.45, green: 0.28, blue: 0.85, alpha: 1.0)
        btn.layer.cornerRadius = 8
        return btn
    }()

    private let autoSyncLabel: UILabel = {
        let l = UILabel()
        l.translatesAutoresizingMaskIntoConstraints = false
        l.font = UIFont(name: "Menlo", size: 11) ?? UIFont.systemFont(ofSize: 11)
        l.textColor = UIColor(white: 0.80, alpha: 1.0)
        l.numberOfLines = 0
        l.text = "Axis: Default (-y,-x,-z) | Offset: 0.0 ms"
        return l
    }()

    private let pitchLabel = UILabel()
    private let pitchSlider = UISlider()

    private let yawLabel = UILabel()
    private let yawSlider = UISlider()

    private let rollLabel = UILabel()
    private let rollSlider = UISlider()

    private let zoomLabel = UILabel()
    private let zoomSlider = UISlider()
    private let unlimitedZoomSwitch = UISwitch()

    private let dynamicZoomLabel = UILabel()
    private let dynamicZoomSwitch = UISwitch()

    private let horizonLabel = UILabel()
    private let horizonSwitch = UISwitch()
    private let tiltLabel = UILabel()
    private let tiltSlider = UISlider()

    private let rsLabel = UILabel()
    private let rsSwitch = UISwitch()

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
        for lbl in [pitchLabel, yawLabel, rollLabel, zoomLabel, dynamicZoomLabel, horizonLabel, tiltLabel, rsLabel, offsetLabel] {
            lbl.translatesAutoresizingMaskIntoConstraints = false
            lbl.font = UIFont(name: "Menlo-Bold", size: 11) ?? UIFont.systemFont(ofSize: 11, weight: .bold)
            lbl.textColor = .white
        }

        configureSlider(pitchSlider, min: 0.0, max: 1.0, val: 0.65)
        configureSlider(yawSlider, min: 0.0, max: 1.0, val: 0.65)
        configureSlider(rollSlider, min: 0.0, max: 1.0, val: 0.75)
        configureSlider(zoomSlider, min: 1.05, max: 1.60, val: 1.35)
        configureSlider(tiltSlider, min: -15.0, max: 15.0, val: 0.0)
        configureSlider(offsetSlider, min: -80.0, max: 80.0, val: 0.0)

        unlimitedZoomSwitch.translatesAutoresizingMaskIntoConstraints = false
        unlimitedZoomSwitch.isOn = false
        unlimitedZoomSwitch.addTarget(self, action: #selector(handleSliderChanged), for: .valueChanged)

        dynamicZoomSwitch.translatesAutoresizingMaskIntoConstraints = false
        dynamicZoomSwitch.isOn = true
        dynamicZoomSwitch.addTarget(self, action: #selector(handleSliderChanged), for: .valueChanged)

        horizonSwitch.translatesAutoresizingMaskIntoConstraints = false
        horizonSwitch.isOn = false
        horizonSwitch.addTarget(self, action: #selector(handleSliderChanged), for: .valueChanged)

        rsSwitch.translatesAutoresizingMaskIntoConstraints = false
        rsSwitch.isOn = true
        rsSwitch.addTarget(self, action: #selector(handleSliderChanged), for: .valueChanged)

        autoSyncButton.addTarget(self, action: #selector(handleAutoSync), for: .touchUpInside)
        previewButton.addTarget(self, action: #selector(handlePreview), for: .touchUpInside)
        renderButton.addTarget(self, action: #selector(handleRenderTap), for: .touchUpInside)
        playButton.addTarget(self, action: #selector(handlePlay), for: .touchUpInside)
        savePhotosButton.addTarget(self, action: #selector(handleSavePhotos), for: .touchUpInside)
        shareButton.addTarget(self, action: #selector(handleShare), for: .touchUpInside)
    }

    private func configureSlider(_ s: UISlider, min: Float, max: Float, val: Float) {
        s.translatesAutoresizingMaskIntoConstraints = false
        s.minimumValue = min
        s.maximumValue = max
        s.value = val
        s.addTarget(self, action: #selector(handleSliderChanged), for: .valueChanged)
    }

    private func makeSwitchRow(_ label: UILabel, _ toggle: UISwitch) -> UIStackView {
        let row = UIStackView(arrangedSubviews: [label, toggle])
        row.translatesAutoresizingMaskIntoConstraints = false
        row.axis = .horizontal
        row.alignment = .center
        row.spacing = 8
        return row
    }

    private func setupLayout() {
        view.addSubview(scrollView)
        scrollView.addSubview(contentView)

        let zoomRow = makeSwitchRow(zoomLabel, unlimitedZoomSwitch)
        let dynZoomRow = makeSwitchRow(dynamicZoomLabel, dynamicZoomSwitch)
        let horizonRow = makeSwitchRow(horizonLabel, horizonSwitch)
        let rsRow = makeSwitchRow(rsLabel, rsSwitch)

        let stack = UIStackView(arrangedSubviews: [
            autoSyncButton,
            autoSyncLabel,
            pitchLabel,
            pitchSlider,
            yawLabel,
            yawSlider,
            rollLabel,
            rollSlider,
            zoomRow,
            zoomSlider,
            dynZoomRow,
            horizonRow,
            tiltLabel,
            tiltSlider,
            rsRow,
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
        stack.spacing = 9
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

            autoSyncButton.heightAnchor.constraint(equalToConstant: 40),
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
            pitchSmoothness: Double(pitchSlider.value),
            yawSmoothness: Double(yawSlider.value),
            rollSmoothness: Double(rollSlider.value),
            velocityDamping: 1.0,
            timeOffsetMs: Double(offsetSlider.value),
            maxZoomLimit: maxZoom,
            horizonLockEnabled: horizonSwitch.isOn,
            horizonLockAmount: 1.0,
            horizonTiltOffsetDeg: Double(tiltSlider.value),
            rollingShutterCorrection: rsSwitch.isOn,
            dynamicZoomEnabled: dynamicZoomSwitch.isOn,
            axisPresetIndex: detectedAxisPresetIndex
        )
    }

    private func updateSliderLabels() {
        pitchLabel.text = String(format: "Pitch Smoothness: %.2f", pitchSlider.value)
        yawLabel.text = String(format: "Yaw Smoothness:   %.2f", yawSlider.value)
        rollLabel.text = String(format: "Roll Smoothness:  %.2f", rollSlider.value)

        if unlimitedZoomSwitch.isOn {
            zoomLabel.text = "Max Zoom: UNLIMITED (0% Border)"
            zoomSlider.isEnabled = false
            zoomSlider.alpha = 0.4
        } else {
            zoomLabel.text = String(format: "Max Zoom Limit: %.2fx", zoomSlider.value)
            zoomSlider.isEnabled = true
            zoomSlider.alpha = 1.0
        }

        dynamicZoomLabel.text = dynamicZoomSwitch.isOn ? "Dynamic Zoom: ON" : "Dynamic Zoom: STATIC"
        horizonLabel.text = horizonSwitch.isOn ? "360° Horizon Lock: ON" : "360° Horizon Lock: OFF"
        tiltLabel.text = String(format: "Horizon Tilt: %+.1f deg", tiltSlider.value)
        tiltSlider.isEnabled = horizonSwitch.isOn
        tiltSlider.alpha = horizonSwitch.isOn ? 1.0 : 0.4

        rsLabel.text = rsSwitch.isOn ? "Rolling Shutter Fix: ON" : "Rolling Shutter Fix: OFF"
        offsetLabel.text = String(format: "IMU Sync Offset: %+.1f ms", offsetSlider.value)
    }

    @objc private func handleSliderChanged() {
        updateSliderLabels()
    }

    @objc private func handleAutoSync() {
        autoSyncButton.isEnabled = false
        autoSyncButton.alpha = 0.6
        autoSyncLabel.textColor = UIColor(white: 0.9, alpha: 1.0)
        autoSyncLabel.text = "Running Optical Flow Auto-Sync & Axis Verification..."

        pipeline.runAutoSync(take: take) { [weak self] result in
            guard let self = self else { return }
            self.autoSyncButton.isEnabled = true
            self.autoSyncButton.alpha = 1.0

            switch result {
            case .success(let res):
                if res.valid && res.bestNCC >= 0.35 {
                    self.detectedAxisPresetIndex = res.bestAxisPresetIndex
                    self.offsetSlider.value = Float(res.bestOffsetMs)
                    self.updateSliderLabels()
                    self.autoSyncLabel.textColor = UIColor(red: 0.35, green: 0.95, blue: 0.55, alpha: 1.0)
                    self.autoSyncLabel.text = res.diagnosticMessage
                    self.handlePreview()
                } else {
                    self.autoSyncLabel.textColor = UIColor(red: 1.0, green: 0.78, blue: 0.25, alpha: 1.0)
                    self.autoSyncLabel.text = res.diagnosticMessage
                }
            case .failure(let err):
                self.autoSyncLabel.textColor = UIColor(red: 1.0, green: 0.45, blue: 0.45, alpha: 1.0)
                self.autoSyncLabel.text = "Auto-Sync Error: \(err.localizedDescription)"
            }
        }
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
        statusLabel.text = "Initializing Metal Lanczos4 + RS pipeline..."

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
