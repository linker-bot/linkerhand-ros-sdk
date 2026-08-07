from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = "linkerbot_hand_ros"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("tests",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "linkerbot>=0.5.4"],
    python_requires=">=3.10",
    zip_safe=True,
    maintainer="zhuzhixiang",
    maintainer_email="1849346915@qq.com",
    description="ROS 2 driver for Linkerbot dexterous hands",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "hand_node = linkerbot_hand_ros.node:main",
        ],
    },
)
