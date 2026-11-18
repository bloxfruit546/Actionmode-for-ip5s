import UIKit

final class QueueViewController: UITableViewController {
    override func viewDidLoad() {
        super.viewDidLoad()
        navigationItem.title = "Render Queue"
        tableView.backgroundColor = UIColor(red: 0.09, green: 0.10, blue: 0.12, alpha: 1.0)
        tableView.separatorColor = UIColor(white: 0.22, alpha: 1.0)
        tableView.register(UITableViewCell.self, forCellReuseIdentifier: "QueueCell")

        navigationItem.leftBarButtonItem = UIBarButtonItem(
            title: "Clear",
            style: .plain,
            target: self,
            action: #selector(handleClear)
        )
        navigationItem.rightBarButtonItem = UIBarButtonItem(
            title: "Resume",
            style: .plain,
            target: self,
            action: #selector(handleResume)
        )

        NotificationCenter.default.addObserver(
            self,
            selector: #selector(handleQueueUpdated),
            name: RenderQueueManager.didUpdateNotification,
            object: nil
        )
    }

    deinit {
        NotificationCenter.default.removeObserver(self)
    }

    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        tableView.reloadData()
    }

    @objc private func handleQueueUpdated() {
        tableView.reloadData()
    }

    @objc private func handleResume() {
        RenderQueueManager.shared.resumePendingJobs()
    }

    @objc private func handleClear() {
        RenderQueueManager.shared.clearCompleted()
    }

    override func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        return RenderQueueManager.shared.jobs.count
    }

    override func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let cell = UITableViewCell(style: .subtitle, reuseIdentifier: "QueueCell")
        let job = RenderQueueManager.shared.jobs[indexPath.row]
        cell.backgroundColor = UIColor(red: 0.12, green: 0.13, blue: 0.16, alpha: 1.0)
        cell.textLabel?.font = UIFont(name: "Menlo-Bold", size: 12) ?? UIFont.systemFont(ofSize: 12, weight: .bold)
        cell.detailTextLabel?.font = UIFont(name: "Menlo", size: 11) ?? UIFont.systemFont(ofSize: 11)

        switch job.status {
        case .completed:
            cell.textLabel?.textColor = UIColor(red: 0.3, green: 0.9, blue: 0.45, alpha: 1.0)
        case .failed:
            cell.textLabel?.textColor = UIColor(red: 1.0, green: 0.4, blue: 0.4, alpha: 1.0)
        case .running:
            cell.textLabel?.textColor = UIColor(red: 0.35, green: 0.78, blue: 1.0, alpha: 1.0)
        case .paused:
            cell.textLabel?.textColor = UIColor(red: 1.0, green: 0.78, blue: 0.25, alpha: 1.0)
        case .pending:
            cell.textLabel?.textColor = .white
        }

        cell.textLabel?.text = "[\(job.status.rawValue.uppercased())] \(job.takeUUID.prefix(8))"
        if job.status == .failed, let err = job.errorMessage {
            cell.detailTextLabel?.textColor = UIColor(red: 1.0, green: 0.65, blue: 0.65, alpha: 1.0)
            cell.detailTextLabel?.text = err
        } else {
            cell.detailTextLabel?.textColor = UIColor(white: 0.75, alpha: 1.0)
            cell.detailTextLabel?.text = String(
                format: "%.0f%% (%d/%d frames)  Zoom: %@",
                job.progress * 100.0,
                job.completedFrames,
                job.totalFrames,
                job.config.maxZoomLimit <= 1.0 ? "Unlimited" : String(format: "%.2fx", job.config.maxZoomLimit)
            )
        }
        return cell
    }

    override func tableView(
        _ tableView: UITableView,
        commit editingStyle: UITableViewCell.EditingStyle,
        forRowAt indexPath: IndexPath
    ) {
        if editingStyle == .delete {
            RenderQueueManager.shared.removeJob(at: indexPath.row)
        }
    }
}
