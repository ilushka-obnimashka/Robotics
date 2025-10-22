import os
from glob import glob
from setuptools import find_packages, setup

package_name = "ex02"

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
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "carrot_follower = ex2.carrot_follower:main",
            "carrot_broadcaster = ex2.dynamic_carrot_frame_broadcaster:main",
            "turtle_tf2_broadcaster = ex2.turtle_tf2_broadcaster:main",
        ],
    },
)
