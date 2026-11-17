import UIKit

@UIApplicationMain
final class AppDelegate: UIResponder, UIApplicationDelegate {
    var window: UIWindow?

    func application(
        _ application: UIApplication,
        didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?
    ) -> Bool {
        UIApplication.shared.isIdleTimerDisabled = true

        let win = UIWindow(frame: UIScreen.main.bounds)
        win.rootViewController = MainTabBarController()
        win.makeKeyAndVisible()
        self.window = win
        return true
    }
}
