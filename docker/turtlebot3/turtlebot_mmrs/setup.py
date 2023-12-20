import os
from glob import glob
from setuptools import setup

package_name = "turtlebot_mmrs"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "maps"), glob("maps/*.pgm")),
        (os.path.join("share", package_name, "maps"), glob("maps/*.yaml")),
        (os.path.join("share", package_name, "params"), glob("params/*.yaml")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
    ],
    install_requires=["setuptools", "Shapely"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="mmichal.trela@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot1_navigate = turtlebot_mmrs.robot1_navigate:main",
            "robot2_navigate = turtlebot_mmrs.robot2_navigate:main",
            "controller = turtlebot_mmrs.controller:main",
            (
                "rigid_bodies_listener ="
                " turtlebot_mmrs.rigid_bodies_listener:main"
            ),
        ],
    },
)
