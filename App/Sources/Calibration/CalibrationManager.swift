import CoreMotion
import Foundation

struct CalibrationData: Codable {
    var gyroBiasX: Double
    var gyroBiasY: Double
    var gyroBiasZ: Double
    var gravityX: Double
    var gravityY: Double
    var gravityZ: Double
    var readout1080p30Sec: Double
    var readout720p120Sec: Double
    var fx1080: Double
    var fy1080: Double
    var cx1080: Double
    var cy1080: Double
    var k1: Double
    var k2: Double
    var p1: Double
    var p2: Double
    var calibratedAt: Date?
    var maxGyroStdDev: Double?

    static func defaultIPhone5s() -> CalibrationData {
        return CalibrationData(
            gyroBiasX: 0.0,
            gyroBiasY: 0.0,
            gyroBiasZ: 0.0,
            gravityX: 0.0,
            gravityY: -1.0,
            gravityZ: 0.0,
            readout1080p30Sec: 0.0152,
            readout720p120Sec: 0.0074,
            fx1080: 1815.0,
            fy1080: 1815.0,
            cx1080: 960.0,
            cy1080: 540.0,
            k1: 0.012,
            k2: -0.022,
            p1: 0.0,
            p2: 0.0,
            calibratedAt: nil,
            maxGyroStdDev: nil
        )
    }
}

enum CalibrationError: LocalizedError {
    case imuUnavailable
    case motionDetected(stdDev: Double)
    case insufficientSamples

    var errorDescription: String? {
        switch self {
        case .imuUnavailable:
            return "Gyroscope or Accelerometer is unavailable on this device."
        case .motionDetected(let stdDev):
            return String(format: "Motion detected (stddev=%.4f rad/s > 0.015). Place iPhone flat and still.", stdDev)
        case .insufficientSamples:
            return "Insufficient IMU samples collected."
        }
    }
}

final class CalibrationManager {
    static let shared = CalibrationManager()

    private(set) var current: CalibrationData
    private let queue = DispatchQueue(label: "com.gyrostabilizer5s.calibration")
    private var isCalibrating = false

    private var profileURL: URL {
        let docs = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
        return docs.appendingPathComponent("calibration_profile.json")
    }

    private init() {
        if let data = try? Data(contentsOf: FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0].appendingPathComponent("calibration_profile.json")),
           let decoded = try? JSONDecoder().decode(CalibrationData.self, from: data) {
            self.current = decoded
        } else {
            self.current = CalibrationData.defaultIPhone5s()
        }
    }

    func resetToDefaults() {
        current = CalibrationData.defaultIPhone5s()
        save(current)
    }

    func save(_ profile: CalibrationData) {
        current = profile
        if let encoded = try? JSONEncoder().encode(profile) {
            try? encoded.write(to: profileURL, options: .atomic)
        }
    }

    func runStaticCalibration(
        durationSec: TimeInterval = 8.0,
        progress: @escaping (Double) -> Void,
        completion: @escaping (Result<CalibrationData, Error>) -> Void
    ) {
        queue.async { [weak self] in
            guard let self = self else { return }
            guard !self.isCalibrating else { return }
            self.isCalibrating = true

            let manager = CMMotionManager()
            guard manager.isGyroAvailable && manager.isAccelerometerAvailable else {
                self.isCalibrating = false
                DispatchQueue.main.async {
                    completion(.failure(CalibrationError.imuUnavailable))
                }
                return
            }

            let calibrator = GFGyroBiasCalibratorBridge()
            calibrator.reset()

            let opQueue = OperationQueue()
            opQueue.maxConcurrentOperationCount = 1

            let lock = NSLock()
            var latestAccel = CMAcceleration(x: 0.0, y: -1.0, z: 0.0)

            manager.accelerometerUpdateInterval = 1.0 / 120.0
            manager.gyroUpdateInterval = 1.0 / 120.0

            manager.startAccelerometerUpdates(to: opQueue) { data, _ in
                guard let data = data else { return }
                lock.lock()
                latestAccel = data.acceleration
                lock.unlock()
            }

            manager.startGyroUpdates(to: opQueue) { data, _ in
                guard let data = data else { return }
                lock.lock()
                let acc = latestAccel
                calibrator.addSampleGx(
                    data.rotationRate.x,
                    gy: data.rotationRate.y,
                    gz: data.rotationRate.z,
                    ax: acc.x,
                    ay: acc.y,
                    az: acc.z
                )
                lock.unlock()
            }

            let steps = 40
            let stepInterval = durationSec / Double(steps)
            for i in 1...steps {
                Thread.sleep(forTimeInterval: stepInterval)
                let p = Double(i) / Double(steps)
                DispatchQueue.main.async {
                    progress(p)
                }
            }

            manager.stopGyroUpdates()
            manager.stopAccelerometerUpdates()
            opQueue.waitUntilAllOperationsAreFinished()

            let result = calibrator.finalize(withMaxStdDev: 0.015)
            self.isCalibrating = false

            if result.status == 0 {
                var updated = self.current
                updated.gyroBiasX = result.biasX
                updated.gyroBiasY = result.biasY
                updated.gyroBiasZ = result.biasZ
                updated.gravityX = result.gravityX
                updated.gravityY = result.gravityY
                updated.gravityZ = result.gravityZ
                updated.calibratedAt = Date()
                updated.maxGyroStdDev = result.maxGyroStdDev
                self.save(updated)
                FileLogger.shared.log(String(format: "Static calibration PASS: bias=(%.5f, %.5f, %.5f) stddev=%.5f samples=%llu",
                                             result.biasX, result.biasY, result.biasZ, result.maxGyroStdDev, result.sampleCount))
                DispatchQueue.main.async {
                    completion(.success(updated))
                }
            } else if result.status == 2 {
                FileLogger.shared.log(String(format: "Static calibration REJECTED: stddev=%.5f", result.maxGyroStdDev))
                DispatchQueue.main.async {
                    completion(.failure(CalibrationError.motionDetected(stdDev: result.maxGyroStdDev)))
                }
            } else {
                DispatchQueue.main.async {
                    completion(.failure(CalibrationError.insufficientSamples))
                }
            }
        }
    }
}
