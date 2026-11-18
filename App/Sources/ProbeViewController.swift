import UIKit

final class ProbeViewController: UIViewController {
    private let probeRunner = HardwareProbeRunner()
    private var ramTimer: Timer?

    private let ramLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont(name: "Menlo-Bold", size: 13) ?? UIFont.monospacedDigitSystemFont(ofSize: 13, weight: .bold)
        label.textColor = UIColor(red: 0.35, green: 0.95, blue: 0.55, alpha: 1.0)
        label.text = "RAM: --.- MB | A7 iOS 12"
        return label
    }()

    private let docsPathLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont(name: "Menlo", size: 10) ?? UIFont.systemFont(ofSize: 10)
        label.textColor = UIColor(white: 0.65, alpha: 1.0)
        label.numberOfLines = 2
        label.lineBreakMode = .byTruncatingMiddle
        return label
    }()

    private let runButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("RUN FULL 5S HARDWARE PROBE", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 15, weight: .bold)
        btn.backgroundColor = UIColor(red: 0.12, green: 0.48, blue: 0.95, alpha: 1.0)
        btn.layer.cornerRadius = 8
        return btn
    }()

    private let shareButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("EXPORT / SHARE LOG", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 13, weight: .semibold)
        btn.backgroundColor = UIColor(white: 0.22, alpha: 1.0)
        btn.layer.cornerRadius = 8
        return btn
    }()

    private let clearButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("CLEAR", for: .normal)
        btn.setTitleColor(UIColor(red: 1.0, green: 0.45, blue: 0.45, alpha: 1.0), for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 13, weight: .semibold)
        btn.backgroundColor = UIColor(white: 0.18, alpha: 1.0)
        btn.layer.cornerRadius = 8
        return btn
    }()

    private let logTextView: UITextView = {
        let tv = UITextView()
        tv.translatesAutoresizingMaskIntoConstraints = false
        tv.backgroundColor = UIColor(red: 0.06, green: 0.07, blue: 0.09, alpha: 1.0)
        tv.textColor = UIColor(white: 0.90, alpha: 1.0)
        tv.font = UIFont(name: "Menlo", size: 11) ?? UIFont.systemFont(ofSize: 11)
        tv.isEditable = false
        tv.layer.cornerRadius = 8
        tv.layer.borderWidth = 1
        tv.layer.borderColor = UIColor(white: 0.2, alpha: 1.0).cgColor
        return tv
    }()

    override func viewDidLoad() {
        super.viewDidLoad()
        navigationItem.title = "GyroStabilizer5s Probe"
        view.backgroundColor = UIColor(red: 0.09, green: 0.10, blue: 0.12, alpha: 1.0)

        setupLayout()
        docsPathLabel.text = "Docs: \(FileLogger.shared.documentsDirectoryPath)"
        logTextView.text = FileLogger.shared.readAllLogs()

        FileLogger.shared.onLogAppended = { [weak self] line in
            self?.appendLogLine(line)
        }

        runButton.addTarget(self, action: #selector(handleRunProbe), for: .touchUpInside)
        shareButton.addTarget(self, action: #selector(handleShareLog), for: .touchUpInside)
        clearButton.addTarget(self, action: #selector(handleClearLog), for: .touchUpInside)

        startRamTimer()
    }

    deinit {
        ramTimer?.invalidate()
    }

    private func setupLayout() {
        let actionRow = UIStackView(arrangedSubviews: [shareButton, clearButton])
        actionRow.translatesAutoresizingMaskIntoConstraints = false
        actionRow.axis = .horizontal
        actionRow.spacing = 8
        actionRow.distribution = .fillProportionally

        clearButton.widthAnchor.constraint(equalToConstant: 80).isActive = true

        view.addSubview(ramLabel)
        view.addSubview(docsPathLabel)
        view.addSubview(runButton)
        view.addSubview(actionRow)
        view.addSubview(logTextView)

        let guide = view.safeAreaLayoutGuide
        NSLayoutConstraint.activate([
            ramLabel.topAnchor.constraint(equalTo: guide.topAnchor, constant: 8),
            ramLabel.leadingAnchor.constraint(equalTo: guide.leadingAnchor, constant: 12),
            ramLabel.trailingAnchor.constraint(equalTo: guide.trailingAnchor, constant: -12),

            docsPathLabel.topAnchor.constraint(equalTo: ramLabel.bottomAnchor, constant: 4),
            docsPathLabel.leadingAnchor.constraint(equalTo: guide.leadingAnchor, constant: 12),
            docsPathLabel.trailingAnchor.constraint(equalTo: guide.trailingAnchor, constant: -12),

            runButton.topAnchor.constraint(equalTo: docsPathLabel.bottomAnchor, constant: 10),
            runButton.leadingAnchor.constraint(equalTo: guide.leadingAnchor, constant: 12),
            runButton.trailingAnchor.constraint(equalTo: guide.trailingAnchor, constant: -12),
            runButton.heightAnchor.constraint(equalToConstant: 46),

            actionRow.topAnchor.constraint(equalTo: runButton.bottomAnchor, constant: 8),
            actionRow.leadingAnchor.constraint(equalTo: guide.leadingAnchor, constant: 12),
            actionRow.trailingAnchor.constraint(equalTo: guide.trailingAnchor, constant: -12),
            actionRow.heightAnchor.constraint(equalToConstant: 38),

            logTextView.topAnchor.constraint(equalTo: actionRow.bottomAnchor, constant: 10),
            logTextView.leadingAnchor.constraint(equalTo: guide.leadingAnchor, constant: 12),
            logTextView.trailingAnchor.constraint(equalTo: guide.trailingAnchor, constant: -12),
            logTextView.bottomAnchor.constraint(equalTo: guide.bottomAnchor, constant: -10)
        ])
    }

    private func startRamTimer() {
        updateRamLabel()
        ramTimer = Timer.scheduledTimer(withTimeInterval: 0.5, repeats: true) { [weak self] _ in
            self?.updateRamLabel()
        }
    }

    private func updateRamLabel() {
        let mb = Double(GFSystemProbe.currentPhysFootprintBytes()) / (1024.0 * 1024.0)
        ramLabel.text = String(format: "phys_footprint: %.1f MB | A7 arm64", mb)
    }

    private func appendLogLine(_ line: String) {
        let current = logTextView.text ?? ""
        logTextView.text = current.isEmpty ? line : (current + "\n" + line)
        let len = logTextView.text.count
        if len > 0 {
            logTextView.scrollRangeToVisible(NSRange(location: len - 1, length: 1))
        }
    }

    @objc private func handleRunProbe() {
        runButton.isEnabled = false
        runButton.alpha = 0.6
        runButton.setTitle("RUNNING PROBES...", for: .normal)

        probeRunner.runAllProbes { [weak self] in
            guard let self = self else { return }
            self.runButton.isEnabled = true
            self.runButton.alpha = 1.0
            self.runButton.setTitle("RUN FULL 5S HARDWARE PROBE", for: .normal)
        }
    }

    @objc private func handleShareLog() {
        FileLogger.shared.presentShareSheet(from: self, sourceView: shareButton)
    }

    @objc private func handleClearLog() {
        FileLogger.shared.clearLogs()
        logTextView.text = ""
    }
}
