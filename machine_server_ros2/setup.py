import os
from glob import glob
from setuptools import find_packages, setup

package_name = "machine_server_ros2"
submodules1 = "machine_server_ros2/hostlinkprotocol"
submodules2 = "machine_server_ros2/pymcprotocol"
submodules3 = "machine_server_ros2/mcprotocol"


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, submodules1, submodules2, submodules3],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["config.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tannhat",
    maintainer_email="nguyentannhat2298@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "machine_server_keyence = machine_server_ros2.machine_server_keyence:main",
            "machine_server_mitsu = machine_server_ros2.machine_server_mitsu:main",
            "machine_state_update_keyence = machine_server_ros2.machine_state_update_keyence:main",
            "machine_state_update_mitsu = machine_server_ros2.machine_state_update_mitsu:main",
        ],
    },
)
