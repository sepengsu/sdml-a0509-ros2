from glob import glob
import os

from setuptools import setup

package_name = "sdml_a0509_camera"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*"))),
        (os.path.join("share", package_name, "meshes"), glob(os.path.join("meshes", "*"))),
        (os.path.join("share", package_name, "rviz"), glob(os.path.join("rviz", "*"))),
        (os.path.join("share", package_name, "urdf"), glob(os.path.join("urdf", "*"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jaenote",
    maintainer_email="na06219@naver.com",
    description="SDML A0509 camera description and launch package.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "camera_topic_relay = sdml_a0509_camera.camera_topic_relay:main",
            "virtual_camera_publisher = sdml_a0509_camera.virtual_camera_publisher:main",
            "v4l2_camera_publisher = sdml_a0509_camera.v4l2_camera_publisher:main",
        ],
    },
)
