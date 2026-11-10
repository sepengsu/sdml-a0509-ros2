from setuptools import setup
from glob import glob
import os

package_name = "sdml_a0509_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.launch.py"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jaenote",
    maintainer_email="na06219@naver.com",
    description="SDML A0509 control integration package.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "pick_and_place_node = sdml_a0509_control.node:main",
            "merge_node = sdml_a0509_control.merge_node:main",
            "jog_string_bridge = sdml_a0509_control.jog_string_bridge:main",
        ],
    },
)

