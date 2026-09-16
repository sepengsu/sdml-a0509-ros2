from setuptools import find_packages, setup
from glob import glob
import os

package_name = "a0509_position_getter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jaewon",
    maintainer_email="na06219@naver.com",
    description="A0509 position getter: node + Tkinter GUI (node loads GUI)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "position_getter_gui = a0509_position_getter.position_getter_node:main",
            "robot_mode_node = a0509_position_getter.robot_mode_node:main",
        ],
    },
)
