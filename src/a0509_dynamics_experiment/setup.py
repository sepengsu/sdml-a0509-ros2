from glob import glob
from setuptools import find_packages, setup

package_name = "a0509_dynamics_experiment"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml") + glob("config/*.json")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "matplotlib>=3.5", "numpy>=1.21", "PyYAML>=6.0", "mujoco>=3.0"],
    extras_require={"video": ["mss>=10.0", "imageio-ffmpeg>=0.5", "Pillow>=9.0"],
                    "ag145": ["fast-simplification>=0.2"]},
    zip_safe=True,
    maintainer="SDML",
    maintainer_email="na06219@naver.com",
    description="A0509 dynamics data collection and safe trajectory preview",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "run_experiment = a0509_dynamics_experiment.run_experiment:main",
        "render_ag145_replay = a0509_dynamics_experiment.video_renderer:main",
        "render_mdpg_replay = a0509_dynamics_experiment.mdpg_video_renderer:main",
    ]},
)
