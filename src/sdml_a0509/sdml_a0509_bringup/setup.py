from setuptools import setup
from glob import glob
import os

package_name = "sdml_a0509_bringup"

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
    description="Bringup launch package for SDML A0509.",
    license="TODO: License declaration",
)

