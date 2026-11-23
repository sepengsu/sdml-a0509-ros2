from setuptools import setup

package_name = "a0509_friction_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_data={package_name: ["config.json", "requirements.txt"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="SDML",
    maintainer_email="na06219@naver.com",
    description="Independent MuJoCo-only A0509 friction simulation and validation",
    license="Apache-2.0",
    entry_points={"console_scripts": ["collect_friction = a0509_friction_sim.entry:main"]},
)
