from setuptools import find_packages, setup
import os
from glob import glob


package_name = "gps"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cprt",
    maintainer_email="softwarelead@curover.ca",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rtcm_pub_node = gps.rtcm_pub_node:main",
            "heading_pub_node = gps.heading_pub_node:main",
        ],
    },
)
