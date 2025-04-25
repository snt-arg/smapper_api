import os
from typing import List, Optional

ROS_INSTALL_DIR = "/opt/ros"


def is_ros_installed() -> bool:
    """Check if ROS is installed on the system.

    Returns:
        bool: True if ROS is installed, False otherwise.
    """
    return os.path.exists(ROS_INSTALL_DIR)


def get_installed_ros_distros() -> List[str]:
    """Get a list of installed ROS distributions.

    Returns:
        List[str]: A list of installed ROS distributions.
    """
    if not is_ros_installed():
        return []

    return os.listdir(ROS_INSTALL_DIR)


def get_installed_ros_distro() -> Optional[str]:
    """Get the first installed ROS distribution.

    Returns:
        Optional[str]: The first installed ROS distribution, or None if none are installed.
    """
    distros = get_installed_ros_distros()
    return distros[0] if len(distros) > 0 else None
