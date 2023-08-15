from setuptools import find_packages, setup

package_name = "rqt_multiplot_bridge"

setup(
    name=package_name,
    version="0.0.2",
    packages=find_packages(exclude=["docker"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/bringup.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nick Lamprianidis",
    maintainer_email="info@nlamprian.me",
    description="rqt_multiplot bridge",
    license="GPLv3",
    entry_points={
        "console_scripts": [],
    },
)
