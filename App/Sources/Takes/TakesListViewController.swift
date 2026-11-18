import AVFoundation
import UIKit

final class TakesListViewController: UITableViewController, UIImagePickerControllerDelegate, UINavigationControllerDelegate {
    private var takes: [CaptureTake] = []
    private var importAlert: UIAlertController?

    private let dateFormatter: DateFormatter = {
        let f = DateFormatter()
        f.dateFormat = "yyyy-MM-dd HH:mm:ss"
        return f
    }()

    override func viewDidLoad() {
        super.viewDidLoad()
        navigationItem.title = "Takes"
        navigationItem.rightBarButtonItem = UIBarButtonItem(
            title: "+ Photos",
            style: .plain,
            target: self,
            action: #selector(handleImportFromPhotos)
        )
        tableView.backgroundColor = UIColor(red: 0.09, green: 0.10, blue: 0.12, alpha: 1.0)
        tableView.separatorColor = UIColor(white: 0.22, alpha: 1.0)
        tableView.register(UITableViewCell.self, forCellReuseIdentifier: "TakeCell")
    }

    @objc private func handleImportFromPhotos() {
        PhotosImporter.requestPhotoPermissionIfNeeded { [weak self] granted in
            guard let self = self else { return }
            if !granted {
                let alert = UIAlertController(
                    title: "Photos Access Needed",
                    message: "Please allow Photos access in Settings to import original videos.",
                    preferredStyle: .alert
                )
                alert.addAction(UIAlertAction(title: "OK", style: .default))
                self.present(alert, animated: true)
                return
            }
            guard UIImagePickerController.isSourceTypeAvailable(.photoLibrary) else { return }
            let picker = UIImagePickerController()
            picker.sourceType = .photoLibrary
            picker.mediaTypes = ["public.movie"]
            picker.videoExportPreset = AVAssetExportPresetPassthrough
            picker.delegate = self
            self.present(picker, animated: true)
        }
    }

    func imagePickerControllerDidCancel(_ picker: UIImagePickerController) {
        picker.dismiss(animated: true)
    }

    func imagePickerController(
        _ picker: UIImagePickerController,
        didFinishPickingMediaWithInfo info: [UIImagePickerController.InfoKey: Any]
    ) {
        picker.dismiss(animated: true) { [weak self] in
            guard let self = self else { return }
            let alert = UIAlertController(
                title: "Importing Original Video",
                message: "Preparing Virtual Gyro...",
                preferredStyle: .alert
            )
            self.importAlert = alert
            self.present(alert, animated: true)

            PhotosImporter.importOriginalVideo(
                fromPickerInfo: info,
                progress: { [weak self] pct, msg in
                    self?.importAlert?.message = String(format: "%.0f%% — %@", pct * 100.0, msg)
                },
                completion: { [weak self] result in
                    guard let self = self else { return }
                    self.importAlert?.dismiss(animated: true) {
                        self.importAlert = nil
                        switch result {
                        case .success:
                            self.reloadTakes()
                        case .failure(let err):
                            let errAlert = UIAlertController(
                                title: "Import Failed",
                                message: err.localizedDescription,
                                preferredStyle: .alert
                            )
                            errAlert.addAction(UIAlertAction(title: "OK", style: .default))
                            self.present(errAlert, animated: true)
                        }
                    }
                }
            )
        }
    }

    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        reloadTakes()
    }

    private func reloadTakes() {
        takes = CaptureTakeStore.listTakes()
        tableView.reloadData()
    }

    override func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        return takes.count
    }

    override func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let cell = UITableViewCell(style: .subtitle, reuseIdentifier: "TakeCell")
        let take = takes[indexPath.row]
        cell.backgroundColor = UIColor(red: 0.12, green: 0.13, blue: 0.16, alpha: 1.0)
        cell.textLabel?.textColor = .white
        cell.textLabel?.font = UIFont(name: "Menlo-Bold", size: 12) ?? UIFont.systemFont(ofSize: 12, weight: .bold)
        cell.detailTextLabel?.textColor = UIColor(white: 0.72, alpha: 1.0)
        cell.detailTextLabel?.font = UIFont(name: "Menlo", size: 11) ?? UIFont.systemFont(ofSize: 11)
        cell.accessoryType = .disclosureIndicator

        let movMB = Double(take.movSizeBytes) / (1024.0 * 1024.0)
        let gfKB = Double(take.sidecarSizeBytes) / 1024.0
        cell.textLabel?.text = "\(dateFormatter.string(from: take.createdAt))  [\(take.uuid.prefix(8))]"
        cell.detailTextLabel?.text = String(format: ".mov: %.1f MB  |  .gfraw: %.1f KB", movMB, gfKB)
        return cell
    }

    override func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        tableView.deselectRow(at: indexPath, animated: true)
        let take = takes[indexPath.row]
        let detailVC = TakeDetailViewController(take: take)
        navigationController?.pushViewController(detailVC, animated: true)
    }

    override func tableView(
        _ tableView: UITableView,
        commit editingStyle: UITableViewCell.EditingStyle,
        forRowAt indexPath: IndexPath
    ) {
        if editingStyle == .delete {
            let take = takes.remove(at: indexPath.row)
            CaptureTakeStore.deleteTake(take)
            tableView.deleteRows(at: [indexPath], with: .automatic)
        }
    }
}
