from setuptools import find_packages, setup

package_name = "compressed_telemetry"

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
            "costmap_compressor = compressed_telemetry.costmap_compressor:main",
            "costmap_decompressor = compressed_telemetry.costmap_decompressor:main",
            "gridmap_compressor = compressed_telemetry.gridmap_compressor:main",
            "gridmap_decompressor = compressed_telemetry.gridmap_decompressor:main",
        ],
    },
)
