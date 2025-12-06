from setuptools import find_packages, setup
import os
from glob import glob

package_name = "aruco_localizer"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "config", "aruco_boards"),
            glob(os.path.join("config", "aruco_boards", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "config", "camera_intrinsics"),
            glob(os.path.join("config", "camera_intrinsics", "*.yaml")),
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="erik",
    maintainer_email="erikcaell@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "aruco_localizer_node = aruco_localizer.aruco_localizer_node:main",
            "aruco_board_detector_node = aruco_localizer.aruco_board_detector_node:main",
            "camera_publisher_node = aruco_localizer.camera_publisher_node:main",
        ],
    },
)
