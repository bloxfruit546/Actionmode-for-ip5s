import Foundation

struct CaptureTake {
    let uuid: String
    let movURL: URL
    let sidecarURL: URL
    let createdAt: Date
    let movSizeBytes: UInt64
    let sidecarSizeBytes: UInt64
}

final class CaptureTakeStore {
    static var capturesDirectory: URL {
        let docs = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
        let dir = docs.appendingPathComponent("Captures", isDirectory: true)
        if !FileManager.default.fileExists(atPath: dir.path) {
            try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true, attributes: nil)
        }
        return dir
    }

    static func makeNewTakePaths() -> (uuid: String, movURL: URL, sidecarURL: URL) {
        let uuid = UUID().uuidString
        let dir = capturesDirectory
        let movURL = dir.appendingPathComponent("\(uuid).mov")
        let sidecarURL = dir.appendingPathComponent("\(uuid).gfraw")
        return (uuid, movURL, sidecarURL)
    }

    static func listTakes() -> [CaptureTake] {
        let dir = capturesDirectory
        guard let items = try? FileManager.default.contentsOfDirectory(
            at: dir,
            includingPropertiesForKeys: [.creationDateKey, .fileSizeKey],
            options: [.skipsHiddenFiles]
        ) else {
            return []
        }

        let movFiles = items.filter { $0.pathExtension.lowercased() == "mov" }
        var takes: [CaptureTake] = []

        for movURL in movFiles {
            let uuid = movURL.deletingPathExtension().lastPathComponent
            let sidecarURL = dir.appendingPathComponent("\(uuid).gfraw")

            let movAttrs = try? FileManager.default.attributesOfItem(atPath: movURL.path)
            let sidecarAttrs = try? FileManager.default.attributesOfItem(atPath: sidecarURL.path)

            let createdAt = (movAttrs?[.creationDate] as? Date) ?? Date.distantPast
            let movSize = (movAttrs?[.size] as? NSNumber)?.uint64Value ?? 0
            let sidecarSize = (sidecarAttrs?[.size] as? NSNumber)?.uint64Value ?? 0

            takes.append(CaptureTake(
                uuid: uuid,
                movURL: movURL,
                sidecarURL: sidecarURL,
                createdAt: createdAt,
                movSizeBytes: movSize,
                sidecarSizeBytes: sidecarSize
            ))
        }

        return takes.sorted { $0.createdAt > $1.createdAt }
    }

    static func deleteTake(_ take: CaptureTake) {
        try? FileManager.default.removeItem(at: take.movURL)
        try? FileManager.default.removeItem(at: take.sidecarURL)
    }
}
