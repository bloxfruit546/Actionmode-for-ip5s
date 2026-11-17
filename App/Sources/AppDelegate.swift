import UIKit

@UIApplicationMain
final class AppDelegate: UIResponder, UIApplicationDelegate {
    var window: UIWindow?

    func application(
        _ application: UIApplication,
        didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?
    ) -> Bool {
        UIApplication.shared.isIdleTimerDisabled = true

        let nav = UINavigationController(rootViewController: ProbeViewController())
        nav.navigationBar.barStyle = .black
        nav.navigationBar.isTranslucent = false
        nav.navigationBar.tintColor = .white

        let win = UIWindow(frame: UIScreen.main.bounds)
        win.rootViewController = nav
        win.makeKeyAndVisible()
        self.window = win
        return true
    }
}
