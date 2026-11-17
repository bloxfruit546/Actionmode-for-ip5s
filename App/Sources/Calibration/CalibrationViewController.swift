import UIKit

final class CalibrationViewController: UIViewController {
    private let infoTextView: UITextView = {
        let tv = UITextView()
        tv.translatesAutoresizingMaskIntoConstraints = false
        tv.backgroundColor = UIColor(red: 0.06, green: 0.07, blue: 0.09, alpha: 1.0)
        tv.textColor = UIColor(white: 0.92, alpha: 1.0)
        tv.font = UIFont(name: "Menlo", size: 12) ?? UIFont.systemFont(ofSize: 12)
        tv.isEditable = false
        tv.layer.cornerRadius = 8
        tv.layer.borderWidth = 1
        tv.layer.borderColor = UIColor(white: 0.22, alpha: 1.0).cgColor
        return tv
    }()

    private let statusLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 13, weight: .medium)
        label.textColor = UIColor(white: 0.85, alpha: 1.0)
        label.numberOfLines = 0
        label.text = "Place iPhone 5s flat on a motionless surface before starting."
        return label
    }()

    private let progressView: UIProgressView = {
        let pv = UIProgressView(progressViewStyle: .default)
        pv.translatesAutoresizingMaskIntoConstraints = false
        pv.progressTintColor = UIColor(red: 0.20, green: 0.78, blue: 0.45, alpha: 1.0)
        pv.trackTintColor = UIColor(white: 0.25, alpha: 1.0)
        return pv
    }()

    private let calibrateButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("START 8s STATIC GYRO CALIBRATION", for: .normal)
        btn.setTitleColor(.white, for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 15, weight: .bold)
        btn.backgroundColor = UIColor(red: 0.12, green: 0.48, blue: 0.95, alpha: 1.0)
        btn.layer.cornerRadius = 8
        return btn
    }()

    private let resetButton: UIButton = {
        let btn = UIButton(type: .system)
        btn.translatesAutoresizingMaskIntoConstraints = false
        btn.setTitle("RESET TO IMX145 DEFAULTS", for: .normal)
        btn.setTitleColor(UIColor(red: 1.0, green: 0.55, blue: 0.35, alpha: 1.0), for: .normal)
        btn.titleLabel?.font = UIFont.systemFont(ofSize: 13, weight: .semibold)
        btn.backgroundColor = UIColor(white: 0.18, alpha: 1.0)
        btn.layer.cornerRadius = 8
        return btn
    }()

    override func viewDidLoad() {
        super.viewDidLoad()
        title = "Calibration"
        view.backgroundColor = UIColor(red: 0.09, green: 0.10, blue: 0.12, alpha: 1.0)

        setupLayout()
        refreshProfileDisplay()

        calibrateButton.addTarget(self, action: #selector(handleCalibrate), for: .touchUpInside)
        resetButton.addTarget(self, action: #selector(handleReset), for: .touchUpInside)
    }

    private func setupLayout() {
        view.addSubview(infoTextView)
        view.addSubview(statusLabel)
        view.addSubview(progressView)
        view.addSubview(calibrateButton)
        view.addSubview(resetButton)

        let guide = view.safeAreaLayoutGuide
        NSLayoutConstraint.activate([
            infoTextView.topAnchor.constraint(equalTo: guide.topAnchor, constant: 12),
            infoTextView.leadingAnchor.constraint(equalTo: guide.leadingAnchor, constant: 12),
            infoTextView.trailingAnchor.constraint(equalTo: guide.trailingAnchor, constant: -12),
            infoTextView.heightAnchor.constraint(equalToConstant: 240),

            statusLabel.topAnchor.constraint(equalTo: infoTextView.bottomAnchor, constant: 14),
            statusLabel.leadingAnchor.constraint(equalTo: guide.leadingAnchor, constant: 12),
            statusLabel.trailingAnchor.constraint(equalTo: guide.trailingAnchor, constant: -12),

            progressView.topAnchor.constraint(equalTo: statusLabel.bottomAnchor, constant: 10),
            progressView.leadingAnchor.constraint(equalTo: guide.leadingAnchor, constant: 12),
            progressView.trailingAnchor.constraint(equalTo: guide.trailingAnchor, constant: -12),

            calibrateButton.topAnchor.constraint(equalTo: progressView.bottomAnchor, constant: 16),
            calibrateButton.leadingAnchor.constraint(equalTo: guide.leadingAnchor, constant: 12),
            calibrateButton.trailingAnchor.constraint(equalTo: guide.trailingAnchor, constant: -12),
            calibrateButton.heightAnchor.constraint(equalToConstant: 48),

            resetButton.topAnchor.constraint(equalTo: calibrateButton.bottomAnchor, constant: 10),
            resetButton.leadingAnchor.constraint(equalTo: guide.leadingAnchor, constant: 12),
            resetButton.trailingAnchor.constraint(equalTo: guide.trailingAnchor, constant: -12),
            resetButton.heightAnchor.constraint(equalToConstant: 40)
        ])
    }

    private func refreshProfileDisplay() {
        let p = CalibrationManager.shared.current
        let dateStr: String
        if let d = p.calibratedAt {
            let fmt = DateFormatter()
            fmt.dateFormat = "yyyy-MM-dd HH:mm:ss"
            dateStr = fmt.string(from: d)
        } else {
            dateStr = "Never (Factory Baseline)"
        }

        infoTextView.text = String(format: """
        iPhone 5s (Sony IMX145) Profile
        ------------------------------------
        Calibrated At : %@
        Gyro Bias X   : %+.6f rad/s
        Gyro Bias Y   : %+.6f rad/s
        Gyro Bias Z   : %+.6f rad/s
        Gravity Vec   : (%.3f, %.3f, %.3f)
        Max StdDev    : %@

        Lens 1080p    : fx=%.1f fy=%.1f cx=%.1f cy=%.1f
        Distortion    : k1=%.4f k2=%.4f p1=%.4f p2=%.4f
        Readout 1080p : %.2f ms
        Readout 720p  : %.2f ms
        """,
        dateStr,
        p.gyroBiasX, p.gyroBiasY, p.gyroBiasZ,
        p.gravityX, p.gravityY, p.gravityZ,
        p.maxGyroStdDev.map { String(format: "%.5f rad/s", $0) } ?? "N/A",
        p.fx1080, p.fy1080, p.cx1080, p.cy1080,
        p.k1, p.k2, p.p1, p.p2,
        p.readout1080p30Sec * 1000.0,
        p.readout720p120Sec * 1000.0)
    }

    @objc private func handleCalibrate() {
        calibrateButton.isEnabled = false
        calibrateButton.alpha = 0.6
        resetButton.isEnabled = false
        progressView.progress = 0.0
        statusLabel.textColor = UIColor(white: 0.9, alpha: 1.0)
        statusLabel.text = "Sampling Raw Gyro & Accel for 8.0s... DO NOT TOUCH DEVICE."

        CalibrationManager.shared.runStaticCalibration(
            durationSec: 8.0,
            progress: { [weak self] p in
                self?.progressView.progress = Float(p)
            },
            completion: { [weak self] result in
                guard let self = self else { return }
                self.calibrateButton.isEnabled = true
                self.calibrateButton.alpha = 1.0
                self.resetButton.isEnabled = true

                switch result {
                case .success:
                    self.statusLabel.textColor = UIColor(red: 0.35, green: 0.95, blue: 0.55, alpha: 1.0)
                    self.statusLabel.text = "Calibration saved successfully!"
                    self.refreshProfileDisplay()
                case .failure(let err):
                    self.statusLabel.textColor = UIColor(red: 1.0, green: 0.45, blue: 0.45, alpha: 1.0)
                    self.statusLabel.text = err.localizedDescription
                }
            }
        )
    }

    @objc private func handleReset() {
        CalibrationManager.shared.resetToDefaults()
        progressView.progress = 0.0
        statusLabel.textColor = UIColor(white: 0.85, alpha: 1.0)
        statusLabel.text = "Reset to Sony IMX145 baseline."
        refreshProfileDisplay()
    }
}
