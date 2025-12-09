from setuptools import setup
import os
from glob import glob

package_name = "localization"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (
            os.path.join("share", package_name, "config", "ekf"),
            glob("config/ekf/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "config", "urdf"),
            glob("config/urdf/*.xml"),
        ),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Connor",
    maintainer_email="Connor@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "imu_filter = localization.imu_filter:main",
            "repub_odom = localization.republish_odometry:main",
            "lidar_mask_tool = localization.lidar_mask_tool:main",
        ],
    },
)
