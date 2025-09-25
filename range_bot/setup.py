from setuptools import find_packages, setup
import os
from glob import glob

package_name = "range_bot"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="davelkan",
    maintainer_email="davelkan@gmail.com",
    description="Simple Range Finder Bot",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "velocity_control = range_bot.velocity_control:main",
            "rangefinder = range_bot.rangefinder:main",
            "activate_estop = range_bot.estop:publish_true",
            "deactivate_estop = range_bot.estop:publish_false",
        ],
    },
)
