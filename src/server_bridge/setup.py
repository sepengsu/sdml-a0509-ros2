from setuptools import find_packages, setup
from glob import glob
import os

package_name = "server_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name), glob("config/scenario_positions.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jaewon",
    maintainer_email="na06219@naver.com",
    description="Python-based server bridge",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "server_bridge_node = server_bridge.bridge_node:main",
        ],
    },
)
