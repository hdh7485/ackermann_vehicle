from pathlib import Path
import unittest
import xml.etree.ElementTree as ElementTree


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
ROS2_PACKAGE = REPOSITORY_ROOT / "ackermann_vehicle_ros2"
SUPPORTED_DISTRIBUTIONS = {"foxy", "humble", "jazzy"}


class Ros2RepositoryContractTests(unittest.TestCase):
    def expected_distribution(self):
        return (ROS2_PACKAGE / "ROS_DISTRO").read_text().strip()

    def test_ros2_package_declares_ament_and_runtime_contract(self):
        package = ElementTree.parse(ROS2_PACKAGE / "package.xml").getroot()
        self.assertEqual(package.attrib["format"], "3")
        dependencies = {element.text for element in package.findall("depend")}
        self.assertTrue({
            "ackermann_msgs",
            "geometry_msgs",
            "launch",
            "launch_ros",
            "rclpy",
            "robot_state_publisher",
        }.issubset(dependencies))
        self.assertEqual(
            package.findtext("buildtool_depend"), "ament_python"
        )

    def test_ros2_launches_use_ros2_nodes_and_installed_description(self):
        command_launch = (ROS2_PACKAGE / "launch" /
                          "cmd_vel_to_ackermann_drive.launch.py").read_text()
        description_launch = (ROS2_PACKAGE / "launch" /
                              "robot_description.launch.py").read_text()
        self.assertIn('package="ackermann_vehicle_ros2"', command_launch)
        self.assertIn("robot_state_publisher", description_launch)
        self.assertIn('"urdf", "em_3905.urdf.xacro"', description_launch)

    def test_smoke_test_exercises_command_and_description_paths(self):
        smoke_test = (REPOSITORY_ROOT / "scripts" /
                      "ros2_smoke_test.sh").read_text()
        self.assertIn("ros2 topic pub --once /cmd_vel", smoke_test)
        self.assertIn("ros2_ackermann_probe.py", smoke_test)
        self.assertIn("robot_description.launch.py", smoke_test)
        self.assertIn("ROS_DISTRO", smoke_test)
        self.assertIn("expected_distro", smoke_test)
        self.assertIn("ROS_DISTRO is ${ROS_DISTRO}", smoke_test)
        self.assertIn("ROS_DOMAIN_ID", smoke_test)
        self.assertIn("for _ in $(seq 1 10)", smoke_test)

    def test_branch_declares_one_supported_distribution(self):
        expected_distribution = self.expected_distribution()
        self.assertIn(expected_distribution, SUPPORTED_DISTRIBUTIONS)

        readme = (REPOSITORY_ROOT / "README.md").read_text()
        self.assertIn(f"--branch {expected_distribution}", readme)

    def test_ci_covers_the_branch_distribution(self):
        expected_distribution = self.expected_distribution()
        workflow = (
            REPOSITORY_ROOT / ".github" / "workflows" / "ros2.yml"
        ).read_text()

        self.assertIn(f"branches: [{expected_distribution}]", workflow)
        self.assertIn(f"ROS_DISTRO: {expected_distribution}", workflow)
        self.assertIn(f"ros:{expected_distribution}-ros-base", workflow)
        self.assertNotIn("matrix:", workflow)

    def test_ament_package_is_not_hidden_from_colcon(self):
        self.assertFalse((ROS2_PACKAGE / "CATKIN_IGNORE").exists())

        workflow = (
            REPOSITORY_ROOT / ".github" / "workflows" / "ros-noetic.yml"
        ).read_text()
        self.assertNotIn("CATKIN_IGNORE", workflow)
