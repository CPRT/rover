from setuptools import find_packages, setup

package_name = "pid_grapher"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "matplotlib"],
    zip_safe=True,
    maintainer="vscode",
    maintainer_email="PSPuttaguna@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pid_grapher=pid_grapher.base_pid_grapher:main",
        ],
    },
)
