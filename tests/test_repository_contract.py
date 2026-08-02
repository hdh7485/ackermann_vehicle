import importlib.util
import math
import sys
import types
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


class RepositoryContractTest(unittest.TestCase):
    def test_package_metadata_is_current(self):
        package_files = sorted(REPO_ROOT.glob("*/package.xml"))
        self.assertEqual(len(package_files), 4)
        for package_file in package_files:
            root = ET.parse(package_file).getroot()
            self.assertEqual(root.findtext("version"), "0.1.5", package_file)
            self.assertEqual(root.findtext("license"), "Apache 2.0", package_file)
            urls = {url.attrib.get("type"): url.text for url in root.findall("url")}
            self.assertEqual(urls["repository"], "https://github.com/hdh7485/ackermann_vehicle.git")
            self.assertEqual(urls["bugtracker"], "https://github.com/hdh7485/ackermann_vehicle/issues")

    def test_noetic_launch_contract(self):
        launch_names = (
            "ackermann_vehicle.launch",
            "ackermann_vehicle_hokuyo.launch",
            "ackermann_vehicle_hokuyo_imu.launch",
            "ackermann_vehicle_noetic.launch",
        )
        for launch_name in launch_names:
            launch_file = REPO_ROOT / "ackermann_vehicle_gazebo/launch" / launch_name
            root = ET.parse(launch_file).getroot()
            args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("arg")}
            self.assertEqual(args["gui"], "true", launch_file)
            self.assertEqual(args["paused"], "false", launch_file)
            self.assertEqual(args["use_sim_time"], "true", launch_file)
            launch_text = launch_file.read_text()
            self.assertIn('command="load"', launch_text, launch_file)
            self.assertIn("--timeout 60", launch_text, launch_file)
            self.assertNotIn("em_3905_joint_ctrlr_params.yaml\"/>", launch_text, launch_file)

        joy_text = (REPO_ROOT / "ackermann_vehicle_gazebo/launch/ackermann_vehicle_joy.launch").read_text()
        self.assertIn('pkg="ackermann_vehicle_navigation"', joy_text)

    def test_noetic_xacro_commands_do_not_use_removed_xacro_py(self):
        for launch_file in (REPO_ROOT / "ackermann_vehicle_description/launch").glob("*.launch"):
            self.assertNotIn("xacro.py", launch_file.read_text(), launch_file)

    def test_cmd_vel_conversion_math_without_ros_runtime(self):
        """The conversion helper remains unit-testable without a ROS master."""
        rospy = types.ModuleType("rospy")
        geometry_msgs = types.ModuleType("geometry_msgs")
        geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
        geometry_msgs_msg.Twist = type("Twist", (), {})
        ackermann_msgs = types.ModuleType("ackermann_msgs")
        ackermann_msgs_msg = types.ModuleType("ackermann_msgs.msg")
        ackermann_msgs_msg.AckermannDrive = type("AckermannDrive", (), {})
        ackermann_msgs_msg.AckermannDriveStamped = type("AckermannDriveStamped", (), {})
        geometry_msgs.msg = geometry_msgs_msg
        ackermann_msgs.msg = ackermann_msgs_msg
        modules = {
            "rospy": rospy,
            "geometry_msgs": geometry_msgs,
            "geometry_msgs.msg": geometry_msgs_msg,
            "ackermann_msgs": ackermann_msgs,
            "ackermann_msgs.msg": ackermann_msgs_msg,
        }
        previous = {name: sys.modules.get(name) for name in modules}
        sys.modules.update(modules)
        try:
            script = REPO_ROOT / "ackermann_vehicle_navigation/scripts/cmd_vel_to_ackermann_drive.py"
            spec = importlib.util.spec_from_file_location("cmd_vel_to_ackermann_drive", script)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            self.assertEqual(module.convert_trans_rot_vel_to_steering_angle(0.0, 1.0, 1.0), 0)
            self.assertEqual(module.convert_trans_rot_vel_to_steering_angle(1.0, 0.0, 1.0), 0)
            self.assertAlmostEqual(
                module.convert_trans_rot_vel_to_steering_angle(1.0, 0.5, 1.0),
                math.atan(0.5),
            )
        finally:
            for name, value in previous.items():
                if value is None:
                    sys.modules.pop(name, None)
                else:
                    sys.modules[name] = value


if __name__ == "__main__":
    unittest.main()
