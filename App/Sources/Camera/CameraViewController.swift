import AVFoundation
import UIKit

final class CameraViewController: UIViewController {
    private let engine = ProCameraEngine()
    private var previewLayer: AVCaptureVideoPreviewLayer?

    private let previewContainer: UIView = {
        let v = UIView()
        v.translatesAutoresizingMaskIntoConstraints = false
        v.backgroundColor = .black
        return v
    }()

    private let topHudLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont(name: "Menlo-Bold", size: 11) ?? UIFont.monospacedDigitSystemFont(ofSize: 11, weight: .bold)
        label.textColor = UIColor(red: 0.35, green: 0.95, blue: 0.55, alpha: 1.0)
        label.backgroundColor = UIColor(white: 0.0, alpha: 0.65)
        label.textAlignment = .center
        label.layer.cornerRadius = 6
        label.clipsToBounds = true
        label.text = "EIS: OFF | ISO: -- | Exp: -- ms | Lens: --"
        return label
    }()

    private let shutterWarningLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont(name: "Menlo-Bold", size: 10) ?? UIFont.monospacedDigitSystemFont(ofSize: 10, weight: .bold)
        label.textColor = UIColor(red: 1.0, green: 0.65, blue: 0.15, alpha: 1.0)
        label.backgroundColor = UIColor(red: 0.25, green: 0.12, blue: 0.0, alpha: 0.80)
        label.textAlignment = .center
        label.layer.cornerRadius = 5
        label.clipsToBounds = true
        label.text = "LOW SHUTTER (MOTION BLUR RISK) - Recommend <=1/120s"
        label.isHidden = true
        return label
    }()

    private let modeControl: UISegmentedControl = {
        let sc = UISegmentedControl(items: ["1080p30", "720p120"])
        sc.translatesAutoresizingMaskIntoConstraints = false
        sc.selectedSegmentIndex = 0
        sc.tintColor = UIColor(red: 0.25, green: 0.65, blue: 1.0, alpha: 1.0)
        sc.backgroundColor = UIColor(white: 0.12, alpha: 0.85)
        return sc
    }()

    private let bitrateControl: UISegmentedControl = {
        let sc = UISegmentedControl(items: ["35M", "45M", "60M"])
        sc.translatesAutoresizingMaskIntoConstraints = false
        sc.selectedSegmentIndex = 1
        sc.tintColor = UIColor(red: 0.95, green: 0.65, blue: 0.20, alpha: 1.0)
        sc.backgroundColor = UIColor(white: 0.12, alpha: 0.85)
        return sc
    }()

    private let focusLockBtn: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("Focus: AUTO", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 11, weight: .semibold)
        btn.backgroundColor = UIColor(white: 0.18, alpha: 0.85)
        btn.layer.cornerRadius = 6
        return btn
    }()

    private let expLockBtn: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("Exp: AUTO", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 11, weight: .semibold)
        btn.backgroundColor = UIColor(white: 0.18, alpha: 0.85)
        btn.layer.cornerRadius = 6
        return btn
    }()

    private let antiBlurBtn: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("AntiBlur: ON", for: .normal)
        btn.setTitleColor(UIColor(red: 0.35, green: 0.95, blue: 0.55, alpha: 1.0), for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 11, weight: .semibold)
        btn.backgroundColor = UIColor(white: 0.18, alpha: 0.85)
        btn.layer.cornerRadius = 6
        return btn
    }()

    private let motoModeBtn: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("MOTO: OFF", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 11, weight: .heavy)
        btn.backgroundColor = UIColor(white: 0.18, alpha: 0.85)
        btn.layer.cornerRadius = 6
        return btn
    }()

    private let motoBannerLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont(name: "Menlo-Bold", size: 9.5) ?? UIFont.systemFont(ofSize: 9.5, weight: .bold)
        label.textColor = UIColor(red: 1.0, green: 0.88, blue: 0.35, alpha: 1.0)
        label.backgroundColor = UIColor(red: 0.55, green: 0.16, blue: 0.05, alpha: 0.88)
        label.textAlignment = .center
        label.layer.cornerRadius = 5
        label.clipsToBounds = true
        label.numberOfLines = 2
        label.text = "Safety: Use Chest-Mount or Passenger Hold. Focus Locked @ Hyperfocal, Shutter <=1/250s."
        label.isHidden = true
        return label
    }()

    private let recStatusLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont(name: "Menlo-Bold", size: 12) ?? UIFont.monospacedDigitSystemFont(ofSize: 12, weight: .bold)
        label.textColor = .white
        label.textAlignment = .center
        label.text = "READY"
        return label
    }()

    private let recordButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("REC", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 16, weight: .heavy)
        btn.backgroundColor = UIColor(red: 0.88, green: 0.15, blue: 0.15, alpha: 1.0)
        btn.layer.cornerRadius = 32
        btn.layer.borderWidth = 3
        btn.layer.borderColor = UIColor.white.cgColor
        return btn
    }()

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .black
        engine.delegate = self

        setupPreview()
        setupLayout()

        modeControl.addTarget(self, action: #selector(handleModeChanged), for: .valueChanged)
        bitrateControl.addTarget(self, action: #selector(handleBitrateChanged), for: .valueChanged)
        focusLockBtn.addTarget(self, action: #selector(handleToggleFocus), for: .touchUpInside)
        expLockBtn.addTarget(self, action: #selector(handleToggleExp), for: .touchUpInside)
        antiBlurBtn.addTarget(self, action: #selector(handleToggleAntiBlur), for: .touchUpInside)
        motoModeBtn.addTarget(self, action: #selector(handleToggleMotoMode), for: .touchUpInside)
        recordButton.addTarget(self, action: #selector(handleRecordTap), for: .touchUpInside)
    }

    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        requestPermissionsAndStart()
    }

    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        if !engine.isRecording {
            engine.stopSession()
        }
    }

    override func viewDidLayoutSubviews() {
        super.viewDidLayoutSubviews()
        previewLayer?.frame = previewContainer.bounds
    }

    private func setupPreview() {
        let layer = AVCaptureVideoPreviewLayer(session: engine.session)
        layer.videoGravity = .resizeAspectFill
        previewContainer.layer.addSublayer(layer)
        previewLayer = layer
    }

    private func setupLayout() {
        view.addSubview(previewContainer)
        view.addSubview(topHudLabel)
        view.addSubview(modeControl)
        view.addSubview(bitrateControl)

        let lockRow = UIStackView(arrangedSubviews: [focusLockBtn, expLockBtn, antiBlurBtn, motoModeBtn])
        lockRow.translatesAutoresizingMaskIntoConstraints = false
        lockRow.axis = .horizontal
        lockRow.spacing = 6
        lockRow.distribution = .fillEqually
        view.addSubview(lockRow)
        view.addSubview(shutterWarningLabel)
        view.addSubview(motoBannerLabel)

        view.addSubview(recStatusLabel)
        view.addSubview(recordButton)

        let guide = view.safeAreaLayoutGuide
        NSLayoutConstraint.activate([
            previewContainer.topAnchor.constraint(equalTo: view.topAnchor),
            previewContainer.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            previewContainer.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            previewContainer.bottomAnchor.constraint(equalTo: view.bottomAnchor),

            topHudLabel.topAnchor.constraint(equalTo: guide.topAnchor, constant: 6),
            topHudLabel.leadingAnchor.constraint(equalTo: guide.leadingAnchor, constant: 8),
            topHudLabel.trailingAnchor.constraint(equalTo: guide.trailingAnchor, constant: -8),
            topHudLabel.heightAnchor.constraint(equalToConstant: 24),

            modeControl.topAnchor.constraint(equalTo: topHudLabel.bottomAnchor, constant: 8),
            modeControl.leadingAnchor.constraint(equalTo: guide.leadingAnchor, constant: 8),
            modeControl.widthAnchor.constraint(equalToConstant: 150),
            modeControl.heightAnchor.constraint(equalToConstant: 28),

            bitrateControl.topAnchor.constraint(equalTo: topHudLabel.bottomAnchor, constant: 8),
            bitrateControl.trailingAnchor.constraint(equalTo: guide.trailingAnchor, constant: -8),
            bitrateControl.widthAnchor.constraint(equalToConstant: 140),
            bitrateControl.heightAnchor.constraint(equalToConstant: 28),

            lockRow.topAnchor.constraint(equalTo: modeControl.bottomAnchor, constant: 8),
            lockRow.leadingAnchor.constraint(equalTo: guide.leadingAnchor, constant: 8),
            lockRow.trailingAnchor.constraint(equalTo: guide.trailingAnchor, constant: -8),
            lockRow.heightAnchor.constraint(equalToConstant: 30),

            shutterWarningLabel.topAnchor.constraint(equalTo: lockRow.bottomAnchor, constant: 6),
            shutterWarningLabel.leadingAnchor.constraint(equalTo: guide.leadingAnchor, constant: 8),
            shutterWarningLabel.trailingAnchor.constraint(equalTo: guide.trailingAnchor, constant: -8),
            shutterWarningLabel.heightAnchor.constraint(equalToConstant: 20),

            motoBannerLabel.topAnchor.constraint(equalTo: shutterWarningLabel.bottomAnchor, constant: 4),
            motoBannerLabel.leadingAnchor.constraint(equalTo: guide.leadingAnchor, constant: 8),
            motoBannerLabel.trailingAnchor.constraint(equalTo: guide.trailingAnchor, constant: -8),
            motoBannerLabel.heightAnchor.constraint(equalToConstant: 28),

            recordButton.bottomAnchor.constraint(equalTo: guide.bottomAnchor, constant: -14),
            recordButton.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            recordButton.widthAnchor.constraint(equalToConstant: 64),
            recordButton.heightAnchor.constraint(equalToConstant: 64),

            recStatusLabel.bottomAnchor.constraint(equalTo: recordButton.topAnchor, constant: -8),
            recStatusLabel.leadingAnchor.constraint(equalTo: guide.leadingAnchor, constant: 12),
            recStatusLabel.trailingAnchor.constraint(equalTo: guide.trailingAnchor, constant: -12)
        ])
    }

    private func requestPermissionsAndStart() {
        AVCaptureDevice.requestAccess(for: .video) { [weak self] granted in
            guard granted else {
                FileLogger.shared.log("Camera permission denied.")
                return
            }
            AVCaptureDevice.requestAccess(for: .audio) { _ in
                DispatchQueue.main.async {
                    self?.engine.startSession()
                }
            }
        }
    }

    @objc private func handleModeChanged() {
        let mode: CaptureMode = (modeControl.selectedSegmentIndex == 0) ? .cinema1080p30 : .highSpeed720p120
        engine.setCaptureMode(mode)
    }

    @objc private func handleBitrateChanged() {
        let preset: BitratePreset
        switch bitrateControl.selectedSegmentIndex {
        case 0: preset = .mbps35
        case 2: preset = .mbps60
        default: preset = .mbps45
        }
        engine.setBitratePreset(preset)
    }

    @objc private func handleToggleFocus() {
        let next = !engine.isFocusLocked
        engine.setFocusLocked(next)
        focusLockBtn.setTitle(next ? "Focus: LOCK" : "Focus: AUTO", for: .normal)
        focusLockBtn.setTitleColor(next ? UIColor(red: 1.0, green: 0.80, blue: 0.25, alpha: 1.0) : .white, for: .normal)
    }

    @objc private func handleToggleExp() {
        let next = !engine.isExposureLocked
        engine.setExposureLocked(next)
        expLockBtn.setTitle(next ? "Exp: LOCK" : "Exp: AUTO", for: .normal)
        expLockBtn.setTitleColor(next ? UIColor(red: 1.0, green: 0.80, blue: 0.25, alpha: 1.0) : .white, for: .normal)
    }

    @objc private func handleToggleAntiBlur() {
        let next = !engine.isAntiBlurCapEnabled
        engine.setAntiBlurShutterCap(next)
        antiBlurBtn.setTitle(next ? "AntiBlur: ON" : "AntiBlur: OFF", for: .normal)
        antiBlurBtn.setTitleColor(next ? UIColor(red: 0.35, green: 0.95, blue: 0.55, alpha: 1.0) : UIColor(white: 0.75, alpha: 1.0), for: .normal)
    }

    @objc private func handleToggleMotoMode() {
        let next = !engine.isMotoModeEnabled
        engine.setMotoMode(next)
        motoModeBtn.setTitle(next ? "MOTO: ON" : "MOTO: OFF", for: .normal)
        motoModeBtn.backgroundColor = next
            ? UIColor(red: 0.90, green: 0.28, blue: 0.12, alpha: 0.95)
            : UIColor(white: 0.18, alpha: 0.85)
        motoBannerLabel.isHidden = !next
        if next {
            focusLockBtn.setTitle("Focus: HYP", for: .normal)
            focusLockBtn.setTitleColor(UIColor(red: 1.0, green: 0.80, blue: 0.25, alpha: 1.0), for: .normal)
            antiBlurBtn.setTitle("AntiBlur: 1/250", for: .normal)
            antiBlurBtn.setTitleColor(UIColor(red: 0.35, green: 0.95, blue: 0.55, alpha: 1.0), for: .normal)
        } else {
            focusLockBtn.setTitle("Focus: AUTO", for: .normal)
            focusLockBtn.setTitleColor(.white, for: .normal)
            antiBlurBtn.setTitle(engine.isAntiBlurCapEnabled ? "AntiBlur: ON" : "AntiBlur: OFF", for: .normal)
        }
    }

    @objc private func handleRecordTap() {
        if engine.isRecording {
            recordButton.isEnabled = false
            engine.stopRecording()
        } else {
            modeControl.isEnabled = false
            bitrateControl.isEnabled = false
            recordButton.setTitle("STOP", for: .normal)
            recordButton.backgroundColor = UIColor(white: 0.25, alpha: 1.0)
            engine.startRecording()
        }
    }
}

extension CameraViewController: ProCameraEngineDelegate {
    func cameraEngineDidUpdateStatus(
        eisOff: Bool,
        iso: Float,
        exposureMs: Double,
        lensPos: Float,
        gyroCount: UInt64,
        frameCount: UInt32,
        recordingDurationSec: Double,
        gyroMagRadSec: Double
    ) {
        let ramMB = Double(GFSystemProbe.currentPhysFootprintBytes()) / (1024.0 * 1024.0)
        topHudLabel.text = String(
            format: "EIS:%@ | ISO:%.0f | %.1fms | L:%.2f | %.0fMB",
            eisOff ? "OFF" : "ON!", iso, exposureMs, lensPos, ramMB
        )

        if engine.isRecording {
            let mins = Int(recordingDurationSec) / 60
            let secs = Int(recordingDurationSec) % 60
            recStatusLabel.textColor = UIColor(red: 1.0, green: 0.35, blue: 0.35, alpha: 1.0)
            recStatusLabel.text = String(format: "REC %02d:%02d | F:%u | IMU:%llu", mins, secs, frameCount, gyroCount)
            shutterWarningLabel.isHidden = !(exposureMs > 16.7 && gyroMagRadSec > 1.5)
        } else {
            shutterWarningLabel.isHidden = true
        }
    }

    func cameraEngineDidFinishRecording(take: CaptureTake?, summary: GFSidecarSummaryObjC?, error: Error?) {
        recordButton.isEnabled = true
        modeControl.isEnabled = true
        bitrateControl.isEnabled = true
        recordButton.setTitle("REC", for: .normal)
        recordButton.backgroundColor = UIColor(red: 0.88, green: 0.15, blue: 0.15, alpha: 1.0)

        if let err = error {
            recStatusLabel.textColor = UIColor(red: 1.0, green: 0.45, blue: 0.45, alpha: 1.0)
            recStatusLabel.text = "Error: \(err.localizedDescription)"
        } else if let s = summary {
            recStatusLabel.textColor = UIColor(red: 0.35, green: 0.95, blue: 0.55, alpha: 1.0)
            recStatusLabel.text = String(format: "Saved %llu frames (%.1fHz IMU, drop:%u)", s.frameCount, s.gyroHz, s.droppedFrames)
        } else {
            recStatusLabel.text = "READY"
        }
    }
}
