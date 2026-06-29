import os
from glob import glob

from setuptools import find_packages, setup

package_name = "waypoint_manager"


def package_files(directory: str):
    paths = []
    for root, _dirs, filenames in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(root, filename))
    return paths


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config/legs", package_files("config/legs")),
        (f"share/{package_name}/config", ["config/vertices.yaml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="philoshan",
    maintainer_email="philoshan@todo.todo",
    description="Factory diagnostics to Nav2 waypoint route manager",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "waypoint_manager_node = waypoint_manager.waypoint_manager_node:main",
        ],
    },
)
