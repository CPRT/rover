from setuptools import find_packages, setup
from glob import glob

package_name = "computer_vision"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
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
            "zed_aruco_detector_node = computer_vision.zed_aruco_detector:main",
            "keyboard_pnp_locator_node = computer_vision.keyboard_pnp_locator:main",
            "nav_marker_locator_node = computer_vision.nav_marker_locator:main",
        ],
    },
)
