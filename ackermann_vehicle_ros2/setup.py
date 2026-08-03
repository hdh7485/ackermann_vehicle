from glob import glob
import os

from setuptools import find_packages, setup


package_name = "ackermann_vehicle_ros2"


setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob("../ackermann_vehicle_description/urdf/*.xacro"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Donghee Han",
    maintainer_email="hdh7485@kaist.ac.kr",
    description="ROS 2 command and robot-description support for ackermann_vehicle.",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cmd_vel_to_ackermann_drive = "
            "ackermann_vehicle_ros2.cmd_vel_to_ackermann_drive:main",
        ],
    },
)
