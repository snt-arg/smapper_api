import os
from typing import List

ROS_INSTALL_DIR = "/opt/ros"


def is_ros_installed() -> bool:
    return os.path.exists(ROS_INSTALL_DIR)


def get_installed_ros_distros() -> List[str]:
    if not is_ros_installed():
        return []

    return os.listdir(ROS_INSTALL_DIR)
