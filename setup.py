from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

PACKAGE_NAME = "linkerbot_hand_ros"
LONG_DESCRIPTION = Path(__file__).with_name("README.md").read_text(encoding="utf-8")


setup(
    name=PACKAGE_NAME,
    version="0.3.1",
    packages=find_packages(exclude=("tests",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (
            f"share/{PACKAGE_NAME}",
            ["package.xml", "README.md", "README_zh.md"],
        ),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
        (f"share/{PACKAGE_NAME}/docs", glob("docs/*.md")),
        (f"share/{PACKAGE_NAME}/docs/zh", glob("docs/zh/*.md")),
        (
            f"share/{PACKAGE_NAME}/docs/zh/test_reports",
            glob("docs/zh/test_reports/*.md"),
        ),
    ],
    install_requires=[
        "setuptools",
        "linkerbot-hand>=0.1.1,<0.2.0",
    ],
    extras_require={"test": ["pytest"]},
    python_requires=">=3.10",
    zip_safe=True,
    maintainer="zhuzhixiang",
    maintainer_email="1849346915@qq.com",
    description="ROS 2 driver for Linkerbot dexterous hands",
    long_description=LONG_DESCRIPTION,
    long_description_content_type="text/markdown",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "hand_node = linkerbot_hand_ros.node:main",
            "angle_command = linkerbot_hand_ros.examples.angle_command:main",
            "angle_controller = linkerbot_hand_ros.examples.angle_controller:main",
            (
                "frequency_benchmark = "
                "linkerbot_hand_ros.examples.frequency_benchmark:main"
            ),
            "state_monitor = linkerbot_hand_ros.examples.state_monitor:main",
        ],
    },
)
