from setuptools import find_packages, setup

package_name = "ros_roboclaw"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="connor",
    maintainer_email="connor.needham2015@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "roboclaw_node = ros_roboclaw.roboclaw_node:main",
            "drill_roboclaw_node = ros_roboclaw.drill_roboclaw_node:main",
            "elevator_roboclaw_node = ros_roboclaw.elevator_roboclaw_node:main",
            "antenna_roboclaw_node = ros_roboclaw.antenna_roboclaw_node:main",
        ],
    },
)
