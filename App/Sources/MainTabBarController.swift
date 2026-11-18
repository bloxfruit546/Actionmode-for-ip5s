import UIKit

final class MainTabBarController: UITabBarController {
    override func viewDidLoad() {
        super.viewDidLoad()

        tabBar.barStyle = .black
        tabBar.isTranslucent = false
        tabBar.tintColor = UIColor(red: 0.25, green: 0.65, blue: 1.0, alpha: 1.0)
        tabBar.unselectedItemTintColor = UIColor(white: 0.55, alpha: 1.0)

        let cameraVC = CameraViewController()
        cameraVC.tabBarItem = UITabBarItem(title: "Cam", image: nil, tag: 0)

        let takesNav = makeDarkNav(root: TakesListViewController(), tabTitle: "Takes", tag: 1)
        let queueNav = makeDarkNav(root: QueueViewController(), tabTitle: "Queue", tag: 2)
        let calibNav = makeDarkNav(root: CalibrationViewController(), tabTitle: "Calib", tag: 3)
        let probeNav = makeDarkNav(root: ProbeViewController(), tabTitle: "Probe", tag: 4)

        let titleAttrs: [NSAttributedString.Key: Any] = [
            .font: UIFont.systemFont(ofSize: 12, weight: .bold)
        ]
        for item in [cameraVC.tabBarItem, takesNav.tabBarItem, queueNav.tabBarItem, calibNav.tabBarItem, probeNav.tabBarItem] {
            item?.setTitleTextAttributes(titleAttrs, for: .normal)
            item?.titlePositionAdjustment = UIOffset(horizontal: 0, vertical: -12)
        }

        viewControllers = [cameraVC, takesNav, queueNav, calibNav, probeNav]
    }

    private func makeDarkNav(root: UIViewController, tabTitle: String, tag: Int) -> UINavigationController {
        let nav = UINavigationController(rootViewController: root)
        nav.navigationBar.barStyle = .black
        nav.navigationBar.isTranslucent = false
        nav.navigationBar.tintColor = .white
        nav.tabBarItem = UITabBarItem(title: tabTitle, image: nil, tag: tag)
        return nav
    }
}
