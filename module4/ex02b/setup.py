import os
from glob import glob
from setuptools import find_packages, setup

package_name = "ex02b"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.py")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ilya-trushkin",
    maintainer_email="i.trushkin@g.nsu.ru",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "target_switcher = ex2b.target_switcher:main",
            "turtle_controller = ex2b.turtle_controller:main",
            "turtle_tf2_broadcaster = ex2b.turtle_tf2_broadcaster:main",
        ],
    },
)
