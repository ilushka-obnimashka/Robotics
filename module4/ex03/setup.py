import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ex03"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ilya-trushkin",
    maintainer_email="i.trushkin@g.nsu.ru",
    description="TODO: Package description",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "turtle_tf2_broadcaster = ex3.turtle_tf2_broadcaster:main",
            "delay_tf2_listener = ex3.delay_tf2_listener:main",
        ],
    },
)
