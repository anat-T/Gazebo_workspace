import os
from glob import glob

from setuptools import find_packages, setup

package_name = "box_pick_place_demo"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="anat",
    maintainer_email="anat19960@gmail.com",
    description=(
        "Two-robot box pick-and-place demo in Gazebo: one robot searches a "
        "wide belief region for the box, the other gets a narrower region "
        "('help') and finds it almost immediately."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "belief_region_node = box_pick_place_demo.belief_region_node:main",
            "search_and_pick_node = box_pick_place_demo.search_and_pick_node:main",
            "ball_region_node = box_pick_place_demo.ball_region_node:main",
            "ball_sort_node = box_pick_place_demo.ball_sort_node:main",
        ],
    },
)
