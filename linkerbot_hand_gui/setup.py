from setuptools import find_packages, setup


PACKAGE_NAME = "linkerbot_hand_gui"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("tests",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (
            f"share/{PACKAGE_NAME}",
            ["package.xml", "README.md", "CHANGELOG.rst"],
        ),
        (f"share/{PACKAGE_NAME}/launch", ["launch/hand_gui.launch.py"]),
        (f"share/{PACKAGE_NAME}/config", ["config/o6.rviz", "config/o6_preview.rviz"]),
        (f"share/{PACKAGE_NAME}/test", ["test/test_o6_mapping.py"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    python_requires=">=3.10",
    zip_safe=True,
    maintainer="zhuzhixiang",
    maintainer_email="1849346915@qq.com",
    description="O6 URDF slider preview and safe ROS 2 hand control bridge",
    license="Apache-2.0",
    url="https://gitea.linkerhub.work/zhuzhixiang/linkerbot-hand-ros",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "o6_gui_bridge = linkerbot_hand_gui.bridge_node:main",
        ],
    },
)
