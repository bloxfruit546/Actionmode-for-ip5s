import UIKit

final class MainTabBarController: UITabBarController {
    override func viewDidLoad() {
        super.viewDidLoad()

        tabBar.barStyle = .black
        tabBar.isTranslucent = false
        tabBar.tintColor = UIColor(red: 0.25, green: 0.65, blue: 1.0, alpha: 1.0)
        tabBar.unselectedItemTintColor = UIColor(white: 0.55, alpha: 1.0)

        let cameraVC = CameraViewController()
        cameraVC.tabBarItem = UITabBarItem(title: "Camera", image: nil, tag: 0)

        let takesNav = makeDarkNav(root: TakesListViewController(), title: "Takes", tag: 1)
        let calibNav = makeDarkNav(root: CalibrationViewController(), title: "Calibrate", tag: 2)
        let probeNav = makeDarkNav(root: ProbeViewController(), title: "Probe", tag: 3)

        let titleAttrs: [NSAttributedString.Key: Any] = [
            .font: UIFont.systemFont(ofSize: 13, weight: .bold)
        ]
        for item in [cameraVC.tabBarItem, takesNav.tabBarItem, calibNav.tabBarItem, probeNav.tabBarItem] {
            item?.setTitleTextAttributes(titleAttrs, for: .normal)
            item?.titlePositionAdjustment = UIOffset(horizontal: 0, vertical: -12)
        }

        viewControllers = [cameraVC, takesNav, calibNav, probeNav]
    }

    private func makeDarkNav(root: UIViewController, title: String, tag: Int) -> UINavigationController {
        let nav = UINavigationController(rootViewController: root)
        nav.navigationBar.barStyle = .black
        nav.navigationBar.isTranslucent = false
        nav.navigationBar.tintColor = .white
        nav.tabBarItem = UITabBarItem(title: title, image: nil, tag: tag)
        return nav
    }
}
