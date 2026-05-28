from setuptools import find_packages, setup
from glob import glob
import os

package_name = "computer_vision"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="erik",
    maintainer_email="erikcaell@gmail.com",
    description="Various computer vision nodes for URC and CIRC.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "zed_aruco_detector_node = computer_vision.zed_aruco_detector:main"
        ],
    },
)
