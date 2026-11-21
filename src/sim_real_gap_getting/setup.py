from glob import glob
from setuptools import setup

package_name = "sim_real_gap_getting"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/examples", glob("examples/*.json")),
        ("share/" + package_name + "/examples/lr", glob("examples/lr/*.json") + glob("examples/lr/*.png")),
        ("share/" + package_name + "/examples/front", glob("examples/front/*.json") + glob("examples/front/*.png")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SDML",
    maintainer_email="na06219@naver.com",
    description="A0509 JSON trajectory simulation GUI and guarded real practice",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "trajectory_gui = sim_real_gap_getting.gui:main",
        "check_real_rt = sim_real_gap_getting.rt_check:main",
        "validate_trajectory = sim_real_gap_getting.trajectory:main",
        "run_real_trajectory = sim_real_gap_getting.real_execute:main",
        "generate_lr_trajectories = sim_real_gap_getting.lr_trajectory:main",
        "generate_front_trajectories = sim_real_gap_getting.front_trajectory:main",
        "audit_front_grippers = sim_real_gap_getting.front_gripper_audit:main",
        "audit_front_excitation = sim_real_gap_getting.front_quality_audit:main",
        "collect_front_sim = sim_real_gap_getting.front_sim_collect:main",
        "collect_front_batch = sim_real_gap_getting.front_sim_batch:main",
        "front_batch_gui = sim_real_gap_getting.front_batch_gui:main",
        "build_front_60th_scene = sim_real_gap_getting.front_60th_scene:main",
        "review_front_real = sim_real_gap_getting.front_real_review:main",
        "front_real_gui = sim_real_gap_getting.front_real_gui:main",
    ]},
)
