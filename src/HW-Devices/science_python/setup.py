from setuptools import find_packages, setup
import os
from glob import glob

package_name = "science_python"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(
        include=["science_python", "science_python.*"], exclude=["test"]
    ),
    include_package_data=True,
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
    maintainer="aydan",
    maintainer_email="aj01cars@outlook.com",
    description="Nodes for science sensors",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "panoramic = science_python.panoramic:main",
        ],
    },
)
