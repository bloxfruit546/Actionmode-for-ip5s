import AVKit
import UIKit

final class StabilizeViewController: UIViewController {
    private let take: CaptureTake
    private let pipeline = OfflineRenderPipeline()
    private var isRendering = false
    private var lastExportedURL: URL?
    private var lastQualityReportURL: URL?
    private var detectedAxisPresetIndex: Int = -1
    private var hasReadoutOverride: Bool = false
    private var detectedSignedReadoutMs: Double = 15.2
    private var detectedFocalScale: Double = 1.0
    private var detectedRollGyroScale: Double = 1.0

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

    private let actionPresetButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("⚡ ACTION MODE (RECOMMENDED)", for: .normal)
        btn.setTitleColor(.black, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 13, weight: .heavy)
        btn.backgroundColor = UIColor(red: 0.20, green: 0.90, blue: 0.70, alpha: 1.0)
        btn.layer.cornerRadius = 8
        return btn
    }()

    private let motoPresetButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("MOTO POV (ACTION)", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 13, weight: .heavy)
        btn.backgroundColor = UIColor(red: 0.90, green: 0.28, blue: 0.12, alpha: 1.0)
        btn.layer.cornerRadius = 8
        return btn
    }()

    private let heavyJoltPresetButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("🥊 HEAVY JOLT (MAX DAMPING)", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 13, weight: .heavy)
        btn.backgroundColor = UIColor(red: 0.85, green: 0.18, blue: 0.42, alpha: 1.0)
        btn.layer.cornerRadius = 8
        return btn
    }()

    private let naturalPresetButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("Natural", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 12, weight: .bold)
        btn.backgroundColor = UIColor(red: 0.22, green: 0.45, blue: 0.72, alpha: 1.0)
        btn.layer.cornerRadius = 8
        return btn
    }()

    private let ultraGimbalButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("Ultra Gimbal", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 12, weight: .bold)
        btn.backgroundColor = UIColor(red: 0.08, green: 0.62, blue: 0.58, alpha: 1.0)
        btn.layer.cornerRadius = 8
        return btn
    }()

    private let steadicamPresetButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("Steadicam Lock", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 12, weight: .heavy)
        btn.backgroundColor = UIColor(red: 0.85, green: 0.35, blue: 0.15, alpha: 1.0)
        btn.layer.cornerRadius = 8
        return btn
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
    private let zoomModeControl: UISegmentedControl = {
        let sc = UISegmentedControl(items: ["Fixed Crop", "Smooth Dynamic"])
        sc.translatesAutoresizingMaskIntoConstraints = false
        sc.selectedSegmentIndex = 0
        sc.tintColor = UIColor(red: 0.30, green: 0.85, blue: 0.55, alpha: 1.0)
        return sc
    }()

    private let cropTipLabel: UILabel = {
        let l = UILabel()
        l.translatesAutoresizingMaskIntoConstraints = false
        l.font = UIFont(name: "Menlo", size: 10) ?? UIFont.systemFont(ofSize: 10)
        l.textColor = UIColor(red: 1.0, green: 0.82, blue: 0.35, alpha: 1.0)
        l.numberOfLines = 0
        l.text = "Tip: Increase Crop (1.45x–1.60x) to absorb large-amplitude jolts completely."
        return l
    }()

    private let horizonLabel = UILabel()
    private let horizonModeControl: UISegmentedControl = {
        let sc = UISegmentedControl(items: ["Off (Smooth)", "Action Level (±40°)", "360° Lock"])
        sc.translatesAutoresizingMaskIntoConstraints = false
        sc.selectedSegmentIndex = 1
        sc.tintColor = UIColor(red: 0.20, green: 0.90, blue: 0.70, alpha: 1.0)
        return sc
    }()
    private let tiltLabel = UILabel()
    private let tiltSlider = UISlider()

    private let rsLabel = UILabel()
    private let rsSwitch = UISwitch()

    private let hybridResidualLabel = UILabel()
    private let hybridResidualSwitch = UISwitch()
    private let trans2DStrengthLabel = UILabel()
    private let trans2DStrengthSlider = UISlider()

    private let motoActionLabel = UILabel()
    private let motoActionSwitch = UISwitch()

    private let refineCalibLabel = UILabel()
    private let refineCalibSwitch = UISwitch()
    private let constrainedPathLabel = UILabel()
    private let constrainedPathSwitch = UISwitch()
    private let calibSummaryLabel: UILabel = {
        let l = UILabel()
        l.translatesAutoresizingMaskIntoConstraints = false
        l.font = UIFont(name: "Menlo", size: 10.5) ?? UIFont.systemFont(ofSize: 10.5)
        l.textColor = UIColor(red: 0.45, green: 0.88, blue: 1.0, alpha: 1.0)
        l.numberOfLines = 0
        l.text = "Closed-Loop Calib: Pending..."
        return l
    }()

    private let finishingHeaderLabel: UILabel = {
        let l = UILabel()
        l.translatesAutoresizingMaskIntoConstraints = false
        l.font = UIFont(name: "Menlo-Bold", size: 11) ?? UIFont.systemFont(ofSize: 11, weight: .heavy)
        l.textColor = UIColor(red: 0.35, green: 0.85, blue: 1.0, alpha: 1.0)
        l.text = "FINISHING & FULL-FRAME POLISH"
        return l
    }()
    private let framingLabel = UILabel()
    private let framingControl: UISegmentedControl = {
        let sc = UISegmentedControl(items: ["Dynamic Crop", "Full-Frame Fill", "Mirror Edge"])
        sc.translatesAutoresizingMaskIntoConstraints = false
        sc.selectedSegmentIndex = 0
        sc.tintColor = UIColor(red: 0.25, green: 0.72, blue: 0.95, alpha: 1.0)
        return sc
    }()
    private let sharpenLabel = UILabel()
    private let sharpenSlider = UISlider()
    private let temporalSharpenLabel = UILabel()
    private let temporalSharpenSwitch = UISwitch()
    private let exportBitrateLabel = UILabel()
    private let exportBitrateControl: UISegmentedControl = {
        let sc = UISegmentedControl(items: ["25M", "35M", "42M"])
        sc.translatesAutoresizingMaskIntoConstraints = false
        sc.selectedSegmentIndex = 1
        sc.tintColor = UIColor(red: 0.95, green: 0.65, blue: 0.20, alpha: 1.0)
        return sc
    }()

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

    private let gpuProofLabel: UILabel = {
        let l = UILabel()
        l.translatesAutoresizingMaskIntoConstraints = false
        l.font = UIFont(name: "Menlo-Bold", size: 11) ?? UIFont.systemFont(ofSize: 11, weight: .bold)
        l.textColor = UIColor(red: 1.0, green: 0.88, blue: 0.20, alpha: 1.0)
        l.numberOfLines = 0
        l.text = "GPU Matrix Proof: Pending..."
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

    private let shareQualityJsonButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("DEBUG: EXPORT QUALITY JSON", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 12, weight: .bold)
        btn.backgroundColor = UIColor(red: 0.22, green: 0.28, blue: 0.38, alpha: 1.0)
        btn.layer.cornerRadius = 8
        btn.isEnabled = false
        btn.alpha = 0.4
        return btn
    }()

    private let addQueueButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("ADD TO RENDER QUEUE", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 13, weight: .bold)
        btn.backgroundColor = UIColor(red: 0.28, green: 0.36, blue: 0.54, alpha: 1.0)
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
        navigationItem.title = "Stabilize"
        view.backgroundColor = UIColor(red: 0.09, green: 0.10, blue: 0.12, alpha: 1.0)

        configureControls()
        setupLayout()
        loadCachedRefinedCalibration()
        updateSliderLabels()
        handlePreview()
        handleAutoSync()
    }

    private func loadCachedRefinedCalibration() {
        let calibURL = CaptureTakeStore.calibMetadataURL(for: take.uuid)
        if let cal = GFRefinedCalibrationObjC.load(fromJSONPath: calibURL.path), cal.valid {
            calibSummaryLabel.text = cal.summaryString
            offsetSlider.value = Float(cal.offsetA)
        }
    }

    private func configureControls() {
        for lbl in [pitchLabel, yawLabel, rollLabel, zoomLabel, dynamicZoomLabel, horizonLabel, tiltLabel, rsLabel, hybridResidualLabel, trans2DStrengthLabel, motoActionLabel, refineCalibLabel, constrainedPathLabel, framingLabel, sharpenLabel, temporalSharpenLabel, exportBitrateLabel, offsetLabel] {
            lbl.translatesAutoresizingMaskIntoConstraints = false
            lbl.font = UIFont(name: "Menlo-Bold", size: 11) ?? UIFont.systemFont(ofSize: 11, weight: .bold)
            lbl.textColor = .white
        }

        configureSlider(pitchSlider, min: 0.0, max: 1.0, val: 0.78)
        configureSlider(yawSlider, min: 0.0, max: 1.0, val: 0.88)
        configureSlider(rollSlider, min: 0.0, max: 1.0, val: 0.85)
        configureSlider(zoomSlider, min: 1.05, max: 2.40, val: 1.32)
        configureSlider(tiltSlider, min: -15.0, max: 15.0, val: 0.0)
        configureSlider(trans2DStrengthSlider, min: 0.0, max: 1.0, val: 0.0)
        configureSlider(sharpenSlider, min: 0.0, max: 1.0, val: 0.45)
        configureSlider(offsetSlider, min: -80.0, max: 80.0, val: 0.0)

        framingControl.addTarget(self, action: #selector(handleFramingChanged), for: .valueChanged)
        exportBitrateControl.addTarget(self, action: #selector(handleSliderChanged), for: .valueChanged)

        unlimitedZoomSwitch.translatesAutoresizingMaskIntoConstraints = false
        unlimitedZoomSwitch.isOn = false
        unlimitedZoomSwitch.addTarget(self, action: #selector(handleSliderChanged), for: .valueChanged)

        zoomModeControl.addTarget(self, action: #selector(handleFramingChanged), for: .valueChanged)
        zoomModeControl.selectedSegmentIndex = 0

        horizonModeControl.addTarget(self, action: #selector(handleHorizonModeChanged), for: .valueChanged)
        horizonModeControl.selectedSegmentIndex = 1

        rsSwitch.translatesAutoresizingMaskIntoConstraints = false
        rsSwitch.isOn = true
        rsSwitch.addTarget(self, action: #selector(handleSliderChanged), for: .valueChanged)

        hybridResidualSwitch.translatesAutoresizingMaskIntoConstraints = false
        hybridResidualSwitch.isOn = false
        hybridResidualSwitch.addTarget(self, action: #selector(handleSliderChanged), for: .valueChanged)

        motoActionSwitch.translatesAutoresizingMaskIntoConstraints = false
        motoActionSwitch.isOn = false
        motoActionSwitch.addTarget(self, action: #selector(handleSliderChanged), for: .valueChanged)

        refineCalibSwitch.translatesAutoresizingMaskIntoConstraints = false
        refineCalibSwitch.isOn = true
        refineCalibSwitch.addTarget(self, action: #selector(handleRefineToggleChanged), for: .valueChanged)

        constrainedPathSwitch.translatesAutoresizingMaskIntoConstraints = false
        constrainedPathSwitch.isOn = true
        constrainedPathSwitch.addTarget(self, action: #selector(handleRefineToggleChanged), for: .valueChanged)

        temporalSharpenSwitch.translatesAutoresizingMaskIntoConstraints = false
        temporalSharpenSwitch.isOn = true
        temporalSharpenSwitch.addTarget(self, action: #selector(handleSliderChanged), for: .valueChanged)

        actionPresetButton.addTarget(self, action: #selector(handleActionModePreset), for: .touchUpInside)
        motoPresetButton.addTarget(self, action: #selector(handleMotoPovPreset), for: .touchUpInside)
        heavyJoltPresetButton.addTarget(self, action: #selector(handleHeavyJoltPreset), for: .touchUpInside)
        naturalPresetButton.addTarget(self, action: #selector(handleNaturalPreset), for: .touchUpInside)
        ultraGimbalButton.addTarget(self, action: #selector(handleUltraGimbalPreset), for: .touchUpInside)
        steadicamPresetButton.addTarget(self, action: #selector(handleSteadicamPreset), for: .touchUpInside)
        autoSyncButton.addTarget(self, action: #selector(handleAutoSync), for: .touchUpInside)
        previewButton.addTarget(self, action: #selector(handlePreview), for: .touchUpInside)
        renderButton.addTarget(self, action: #selector(handleRenderTap), for: .touchUpInside)
        addQueueButton.addTarget(self, action: #selector(handleAddToQueue), for: .touchUpInside)
        playButton.addTarget(self, action: #selector(handlePlay), for: .touchUpInside)
        savePhotosButton.addTarget(self, action: #selector(handleSavePhotos), for: .touchUpInside)
        shareButton.addTarget(self, action: #selector(handleShare), for: .touchUpInside)
        shareQualityJsonButton.addTarget(self, action: #selector(handleShareQualityJson), for: .touchUpInside)
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

        let presetRow = UIStackView(arrangedSubviews: [naturalPresetButton, ultraGimbalButton, steadicamPresetButton])
        presetRow.translatesAutoresizingMaskIntoConstraints = false
        presetRow.axis = .horizontal
        presetRow.spacing = 6
        presetRow.distribution = .fillEqually

        let zoomRow = makeSwitchRow(zoomLabel, unlimitedZoomSwitch)
        let rsRow = makeSwitchRow(rsLabel, rsSwitch)
        let hybridRow = makeSwitchRow(hybridResidualLabel, hybridResidualSwitch)
        let motoRow = makeSwitchRow(motoActionLabel, motoActionSwitch)
        let refineRow = makeSwitchRow(refineCalibLabel, refineCalibSwitch)
        let constrainedRow = makeSwitchRow(constrainedPathLabel, constrainedPathSwitch)
        let temporalSharpenRow = makeSwitchRow(temporalSharpenLabel, temporalSharpenSwitch)

        let stack = UIStackView(arrangedSubviews: [
            actionPresetButton,
            motoPresetButton,
            heavyJoltPresetButton,
            presetRow,
            autoSyncButton,
            autoSyncLabel,
            refineRow,
            calibSummaryLabel,
            constrainedRow,
            pitchLabel,
            pitchSlider,
            yawLabel,
            yawSlider,
            rollLabel,
            rollSlider,
            zoomRow,
            zoomSlider,
            dynamicZoomLabel,
            zoomModeControl,
            cropTipLabel,
            horizonLabel,
            horizonModeControl,
            tiltLabel,
            tiltSlider,
            rsRow,
            hybridRow,
            trans2DStrengthLabel,
            trans2DStrengthSlider,
            motoRow,
            finishingHeaderLabel,
            framingLabel,
            framingControl,
            sharpenLabel,
            sharpenSlider,
            temporalSharpenRow,
            exportBitrateLabel,
            exportBitrateControl,
            offsetLabel,
            offsetSlider,
            previewButton,
            previewInfoLabel,
            gpuProofLabel,
            progressView,
            statusLabel,
            renderButton,
            addQueueButton,
            playButton,
            savePhotosButton,
            shareButton,
            shareQualityJsonButton
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

            motoPresetButton.heightAnchor.constraint(equalToConstant: 42),
            heavyJoltPresetButton.heightAnchor.constraint(equalToConstant: 42),
            naturalPresetButton.heightAnchor.constraint(equalToConstant: 40),
            ultraGimbalButton.heightAnchor.constraint(equalToConstant: 40),
            steadicamPresetButton.heightAnchor.constraint(equalToConstant: 40),
            autoSyncButton.heightAnchor.constraint(equalToConstant: 40),
            previewButton.heightAnchor.constraint(equalToConstant: 38),
            renderButton.heightAnchor.constraint(equalToConstant: 46),
            addQueueButton.heightAnchor.constraint(equalToConstant: 40),
            playButton.heightAnchor.constraint(equalToConstant: 42),
            savePhotosButton.heightAnchor.constraint(equalToConstant: 42),
            shareButton.heightAnchor.constraint(equalToConstant: 42),
            shareQualityJsonButton.heightAnchor.constraint(equalToConstant: 38)
        ])
    }

    private func currentConfig() -> StabilizeExportConfig {
        let maxZoom: Double = unlimitedZoomSwitch.isOn ? 0.0 : Double(zoomSlider.value)
        let edgeFillMode = framingControl.selectedSegmentIndex
        let fovBoost = (edgeFillMode == 0) ? 0.0 : 1.0
        let bitrateMbps: Int
        switch exportBitrateControl.selectedSegmentIndex {
        case 0: bitrateMbps = 25
        case 2: bitrateMbps = 42
        default: bitrateMbps = 35
        }
        let hMode = horizonModeControl.selectedSegmentIndex
        let hEnabled = (hMode != 0)
        return StabilizeExportConfig(
            pitchSmoothness: Double(pitchSlider.value),
            yawSmoothness: Double(yawSlider.value),
            rollSmoothness: Double(rollSlider.value),
            velocityDamping: 1.0,
            timeOffsetMs: Double(offsetSlider.value),
            maxZoomLimit: maxZoom,
            horizonLockEnabled: hEnabled,
            horizonLockAmount: 1.0,
            horizonTiltOffsetDeg: Double(tiltSlider.value),
            horizonMode: hMode,
            rollingShutterCorrection: rsSwitch.isOn,
            dynamicZoomEnabled: zoomModeControl.selectedSegmentIndex == 1,
            axisPresetIndex: detectedAxisPresetIndex,
            hasReadoutOverride: hasReadoutOverride,
            signedReadoutTimeMs: detectedSignedReadoutMs,
            focalScaleOverride: detectedFocalScale,
            rollGyroScaleOverride: detectedRollGyroScale,
            hybridResidualEnabled: hybridResidualSwitch.isOn,
            translational2DStrength: Double(trans2DStrengthSlider.value),
            motoActionMode: motoActionSwitch.isOn,
            edgeFillMode: edgeFillMode,
            fovScaleBoost: fovBoost,
            sharpenStrength: Double(sharpenSlider.value),
            exportBitrateMbps: bitrateMbps,
            useRefinedCalibration: refineCalibSwitch.isOn,
            useConstrainedPath: constrainedPathSwitch.isOn,
            temporalSharpenEnabled: temporalSharpenSwitch.isOn
        )
    }

    @objc private func handleAddToQueue() {
        RenderQueueManager.shared.enqueue(take: take, config: currentConfig())
        let alert = UIAlertController(
            title: "Added to Render Queue",
            message: "Job queued. Switch to the Queue tab to monitor background progress or resume.",
            preferredStyle: .alert
        )
        alert.addAction(UIAlertAction(title: "OK", style: .default))
        present(alert, animated: true)
    }

    @objc private func handleActionModePreset() {
        motoActionSwitch.isOn = false
        pitchSlider.value = 0.78
        yawSlider.value = 0.88
        rollSlider.value = 0.85
        horizonModeControl.selectedSegmentIndex = 1
        hybridResidualSwitch.isOn = false
        trans2DStrengthSlider.value = 0.0
        unlimitedZoomSwitch.isOn = false
        zoomSlider.value = 1.32
        rsSwitch.isOn = true
        zoomModeControl.selectedSegmentIndex = 0
        updateSliderLabels()
        handlePreview()
    }

    @objc private func handleMotoPovPreset() {
        motoActionSwitch.isOn = true
        pitchSlider.value = 0.94
        yawSlider.value = 0.72
        rollSlider.value = 0.98
        horizonModeControl.selectedSegmentIndex = 2
        hybridResidualSwitch.isOn = false
        trans2DStrengthSlider.value = 0.0
        unlimitedZoomSwitch.isOn = false
        zoomSlider.value = 1.65
        rsSwitch.isOn = true
        zoomModeControl.selectedSegmentIndex = 1
        updateSliderLabels()
        handlePreview()
    }

    @objc private func handleHeavyJoltPreset() {
        motoActionSwitch.isOn = false
        pitchSlider.value = 0.92
        yawSlider.value = 0.92
        rollSlider.value = 0.95
        horizonModeControl.selectedSegmentIndex = 1
        hybridResidualSwitch.isOn = false
        trans2DStrengthSlider.value = 0.0
        unlimitedZoomSwitch.isOn = false
        zoomSlider.value = 1.52
        rsSwitch.isOn = true
        zoomModeControl.selectedSegmentIndex = 0
        updateSliderLabels()
        handlePreview()
    }

    @objc private func handleNaturalPreset() {
        motoActionSwitch.isOn = false
        pitchSlider.value = 0.55
        yawSlider.value = 0.55
        rollSlider.value = 0.65
        horizonModeControl.selectedSegmentIndex = 0
        hybridResidualSwitch.isOn = false
        trans2DStrengthSlider.value = 0.0
        unlimitedZoomSwitch.isOn = false
        zoomSlider.value = 1.15
        rsSwitch.isOn = true
        zoomModeControl.selectedSegmentIndex = 1
        updateSliderLabels()
        handlePreview()
    }

    @objc private func handleUltraGimbalPreset() {
        motoActionSwitch.isOn = false
        pitchSlider.value = 0.85
        yawSlider.value = 0.85
        rollSlider.value = 0.92
        horizonModeControl.selectedSegmentIndex = 2
        hybridResidualSwitch.isOn = false
        trans2DStrengthSlider.value = 0.0
        unlimitedZoomSwitch.isOn = true
        zoomSlider.value = 2.00
        rsSwitch.isOn = true
        zoomModeControl.selectedSegmentIndex = 1
        updateSliderLabels()
        handlePreview()
    }

    @objc private func handleSteadicamPreset() {
        motoActionSwitch.isOn = false
        pitchSlider.value = 0.96
        yawSlider.value = 0.96
        rollSlider.value = 0.98
        horizonModeControl.selectedSegmentIndex = 2
        hybridResidualSwitch.isOn = false
        trans2DStrengthSlider.value = 0.0
        unlimitedZoomSwitch.isOn = false
        zoomSlider.value = 2.20
        rsSwitch.isOn = true
        zoomModeControl.selectedSegmentIndex = 1
        updateSliderLabels()
        handlePreview()
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

        dynamicZoomLabel.text = (zoomModeControl.selectedSegmentIndex == 0)
            ? "Zoom Mode: FIXED CROP (0% Breathing)"
            : "Zoom Mode: SMOOTH DYNAMIC (Peak-Hold)"

        switch horizonModeControl.selectedSegmentIndex {
        case 0:
            horizonLabel.text = "Horizon Mode: OFF (Smooth Roll)"
            tiltSlider.isEnabled = false
            tiltSlider.alpha = 0.4
        case 1:
            horizonLabel.text = "Horizon Mode: ACTION LEVEL (±40° Level / Soft-Knee)"
            tiltSlider.isEnabled = true
            tiltSlider.alpha = 1.0
        default:
            horizonLabel.text = "Horizon Mode: 360° FULL HORIZON LOCK"
            tiltSlider.isEnabled = true
            tiltSlider.alpha = 1.0
        }
        tiltLabel.text = String(format: "Horizon Tilt: %+.1f deg", tiltSlider.value)

        rsLabel.text = rsSwitch.isOn ? "Rolling Shutter Fix (64-Slice): ON" : "Rolling Shutter Fix: OFF"
        hybridResidualLabel.text = hybridResidualSwitch.isOn ? "2D Footstep Polish (160x90): ON" : "2D Footstep Polish (160x90): OFF"
        trans2DStrengthLabel.text = String(format: "2D Polish Strength: %.0f%%", trans2DStrengthSlider.value * 100.0)
        trans2DStrengthSlider.isEnabled = hybridResidualSwitch.isOn && !motoActionSwitch.isOn
        trans2DStrengthSlider.alpha = trans2DStrengthSlider.isEnabled ? 1.0 : 0.4
        motoActionLabel.text = motoActionSwitch.isOn ? "MotoPOV Action Mode (42Hz/Far-ROI): ON" : "MotoPOV Action Mode: OFF"
        refineCalibLabel.text = refineCalibSwitch.isOn ? "Closed-Loop Calib Refine (Per-Clip): ON" : "Closed-Loop Calib Refine: OFF (A/B)"
        constrainedPathLabel.text = constrainedPathSwitch.isOn ? "Optimal L1 Path (Lookahead): ON" : "Optimal L1 Path: OFF (v1.5.2 A/B)"
        let framingNames = ["Dynamic Safe Crop", "Temporal Full-Frame (1.00x)", "Mirror Border (1.00x)"]
        let fIdx = max(0, min(2, framingControl.selectedSegmentIndex))
        framingLabel.text = "Framing: \(framingNames[fIdx])"
        sharpenLabel.text = String(format: "Adaptive Sharpening (CAS): %.0f%%", sharpenSlider.value * 100.0)
        temporalSharpenLabel.text = temporalSharpenSwitch.isOn ? "Temporal Sharpen (Sub-1Hz Zoom LPF): ON" : "Temporal Sharpen: OFF (A/B)"
        let tierNames = ["Standard (25M CABAC)", "Cinema Pro (35M CABAC)", "Max A7 (42M Level 4.1)"]
        let bIdx = max(0, min(2, exportBitrateControl.selectedSegmentIndex))
        exportBitrateLabel.text = "Export Bitrate: \(tierNames[bIdx])"
        offsetLabel.text = String(format: "IMU Sync Offset: %+.1f ms", offsetSlider.value)
    }

    @objc private func handleFramingChanged() {
        updateSliderLabels()
        handlePreview()
    }

    @objc private func handleHorizonModeChanged() {
        if horizonModeControl.selectedSegmentIndex == 2 && zoomSlider.value < 1.65 {
            zoomSlider.value = 1.65
        } else if horizonModeControl.selectedSegmentIndex == 1 && zoomSlider.value < 1.25 {
            zoomSlider.value = 1.28
        }
        updateSliderLabels()
        handlePreview()
    }

    @objc private func handleSliderChanged() {
        updateSliderLabels()
    }

    @objc private func handleRefineToggleChanged() {
        updateSliderLabels()
        handlePreview()
    }

    @objc private func handleAutoSync() {
        autoSyncButton.isEnabled = false
        autoSyncButton.alpha = 0.6
        renderButton.isEnabled = false
        renderButton.alpha = 0.6
        autoSyncLabel.textColor = UIColor(white: 0.9, alpha: 1.0)
        autoSyncLabel.text = "Running Optical Flow Auto-Sync & Axis Verification..."

        pipeline.runAutoSync(take: take, motoActionMode: motoActionSwitch.isOn) { [weak self] result in
            guard let self = self else { return }
            switch result {
            case .success(let res):
                if res.valid && res.bestNCC >= 0.35 {
                    self.detectedAxisPresetIndex = res.bestAxisPresetIndex
                    self.hasReadoutOverride = res.hasReadoutEstimate
                    self.detectedSignedReadoutMs = res.signedReadoutTimeMs
                    self.detectedFocalScale = res.hasFocalScaleEstimate ? res.effectiveFocalScale : 1.0
                    self.detectedRollGyroScale = res.hasRollScaleEstimate ? res.rollGyroScale : 1.0
                    self.offsetSlider.value = Float(res.bestOffsetMs)
                    self.updateSliderLabels()
                    self.autoSyncLabel.textColor = UIColor(red: 0.35, green: 0.95, blue: 0.55, alpha: 1.0)
                    self.autoSyncLabel.text = res.diagnosticMessage
                } else {
                    self.autoSyncLabel.textColor = UIColor(red: 1.0, green: 0.78, blue: 0.25, alpha: 1.0)
                    self.autoSyncLabel.text = res.diagnosticMessage
                }
            case .failure(let err):
                self.autoSyncLabel.textColor = UIColor(red: 1.0, green: 0.45, blue: 0.45, alpha: 1.0)
                self.autoSyncLabel.text = "Auto-Sync Error: \(err.localizedDescription)"
            }

            if self.refineCalibSwitch.isOn {
                self.calibSummaryLabel.text = "Closed-Loop Calib: Refining full clip..."
                self.pipeline.runCalibrationRefine(take: self.take, motoActionMode: self.motoActionSwitch.isOn) { [weak self] calRes in
                    guard let self = self else { return }
                    self.autoSyncButton.isEnabled = true
                    self.autoSyncButton.alpha = 1.0
                    self.renderButton.isEnabled = true
                    self.renderButton.alpha = 1.0
                    switch calRes {
                    case .success(let cal):
                        self.calibSummaryLabel.text = cal.summaryString
                        if cal.valid && !cal.usedPriorFallback {
                            self.detectedAxisPresetIndex = cal.axisPresetIndex
                            self.offsetSlider.value = Float(cal.offsetA)
                            self.updateSliderLabels()
                        }
                        self.handlePreview()
                    case .failure(let err):
                        self.calibSummaryLabel.text = "Calib Refine Error: \(err.localizedDescription)"
                        self.handlePreview()
                    }
                }
            } else {
                self.autoSyncButton.isEnabled = true
                self.autoSyncButton.alpha = 1.0
                self.renderButton.isEnabled = true
                self.renderButton.alpha = 1.0
                self.handlePreview()
            }
        }
    }

    @objc private func handlePreview() {
        let cfg = currentConfig()
        if let plan = pipeline.computePlanPreview(take: take, config: cfg), plan.valid {
            previewInfoLabel.textColor = UIColor(red: 0.35, green: 0.95, blue: 0.55, alpha: 1.0)
            previewInfoLabel.text = String(
                format: "Frames: %u (%ux%u, %@)\nStability Gain: %.1f%% | Max Corr: %.1f°\nRoll In: %.1f° -> Roll Out: %.1f°\nSafe Zoom: [%.2fx .. %.2fx] (avg %.2fx)",
                plan.frameCount, plan.width, plan.height, plan.detectedOrientationName,
                plan.stabilityImprovementPct, plan.maxCorrectionDeg,
                plan.maxInputRollDeg, plan.maxOutputRollErrorDeg,
                plan.minZoom, plan.maxZoom, plan.avgZoom
            )
            if plan.isIdentityMatrix {
                gpuProofLabel.textColor = UIColor(red: 1.0, green: 0.35, blue: 0.35, alpha: 1.0)
                gpuProofLabel.text = "WARN (Identity Matrix!): \(plan.gpuMatrixProof)"
            } else {
                gpuProofLabel.textColor = UIColor(red: 1.0, green: 0.88, blue: 0.20, alpha: 1.0)
                gpuProofLabel.text = plan.gpuMatrixProof
            }
            FileLogger.shared.log("Preview: \(plan.gpuMatrixProof) | Axis=\(detectedAxisPresetIndex) | Offset=\(String(format: "%.1f", cfg.timeOffsetMs))ms")
        } else {
            previewInfoLabel.textColor = UIColor(red: 1.0, green: 0.45, blue: 0.45, alpha: 1.0)
            previewInfoLabel.text = "Failed to compute trajectory preview."
            gpuProofLabel.textColor = UIColor(red: 1.0, green: 0.35, blue: 0.35, alpha: 1.0)
            gpuProofLabel.text = "GPU Matrix Proof: Invalid Plan"
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
        lastQualityReportURL = nil
        savePhotosButton.setTitle("SAVE TO PHOTOS", for: .normal)
        savePhotosButton.backgroundColor = UIColor(red: 0.72, green: 0.42, blue: 0.12, alpha: 1.0)
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
            statusUpdate: { [weak self] msg in
                self?.statusLabel.text = msg
            },
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
                    self.lastQualityReportURL = res.qualityReportURL
                    self.progressView.progress = 1.0
                    self.setResultButtonsEnabled(true)
                    self.shareQualityJsonButton.isEnabled = (res.qualityReportURL != nil)
                    self.shareQualityJsonButton.alpha = (res.qualityReportURL != nil) ? 1.0 : 0.4
                    self.statusLabel.textColor = UIColor(red: 0.35, green: 0.95, blue: 0.55, alpha: 1.0)
                    self.statusLabel.text = String(
                        format: "DONE! %d frames in %.1fs (%.1f fps)\nPeak RAM: %.1f MB | Gain: %.1f%% | 2D: %.1fpx (%lu ok)",
                        res.renderedFrames, res.elapsedSeconds,
                        Double(res.renderedFrames) / max(res.elapsedSeconds, 0.01),
                        res.peakRamMB, res.plan.stabilityImprovementPct,
                        res.plan.hybridResidualMaxPx, UInt(res.plan.translational2DValidFrames)
                    )
                case .failure(let err):
                    self.statusLabel.textColor = UIColor(red: 1.0, green: 0.45, blue: 0.45, alpha: 1.0)
                    self.statusLabel.text = "Error: \(err.localizedDescription)"
                }
            }
        )
    }

    private func setResultButtonsEnabled(_ enabled: Bool) {
        for btn in [playButton, savePhotosButton, shareButton, shareQualityJsonButton] {
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
        OfflineRenderPipeline.saveVideoToPhotos(url: url, from: self) { [weak self] (result: Result<Void, Error>) in
            guard let self = self else { return }
            self.savePhotosButton.isEnabled = true
            switch result {
            case .success:
                self.savePhotosButton.setTitle("✓ SAVED TO PHOTOS", for: .normal)
                self.savePhotosButton.backgroundColor = UIColor(red: 0.18, green: 0.68, blue: 0.38, alpha: 1.0)
                self.statusLabel.textColor = UIColor(red: 0.35, green: 0.95, blue: 0.55, alpha: 1.0)
                self.statusLabel.text = "Saved stabilized video to Photos!"
                let alert = UIAlertController(
                    title: "Success",
                    message: "Saved to Photos Successfully!",
                    preferredStyle: .alert
                )
                alert.addAction(UIAlertAction(title: "OK", style: .default))
                self.present(alert, animated: true)
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

    @objc private func handleShareQualityJson() {
        guard let url = lastQualityReportURL else { return }
        let vc = UIActivityViewController(activityItems: [url], applicationActivities: nil)
        if let pop = vc.popoverPresentationController {
            pop.sourceView = shareQualityJsonButton
            pop.sourceRect = shareQualityJsonButton.bounds
        }
        present(vc, animated: true, completion: nil)
    }
}
