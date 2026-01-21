from setuptools import setup
from glob import glob

package_name = "rover_urdf"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob("launch/*.py")),
        ("share/" + package_name + "/urdf/", glob("urdf/*")),
        ("share/" + package_name + "/rviz/", glob("rviz/*")),
        ("share/" + package_name + "/meshes/", glob("meshes/*")),
        ("share/" + package_name + "/config/", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros-industrial",
    maintainer_email="TODO:",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
