# import glob
# import os
from setuptools import setup

package_name = "decawave_driver"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jdelicat",
    maintainer_email="jakub.delicat@pwr.edu.pl",
    description="Decawave wrapper for ROS 2",
    license="Apache License Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "decawave_driver = decawave_driver.decawave_driver:main",
        ],
    },
)
