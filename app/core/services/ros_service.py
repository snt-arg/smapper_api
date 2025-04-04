from typing import Dict, Optional
from .service import ServiceState, Service
import os
from app.logger import logger


class RosService(Service):
    """A service wrapper that launches and manages a ROS2 node or launch file.

    This class integrates with a workspace and environment configuration to run ROS2 components
    either as individual nodes or via launch files. It handles setup sourcing, executable
    resolution, and command building for running the service.
    """

    def __init__(
        self,
        name: str,
        id: str,
        auto_start: bool,
        restart_on_failure: bool,
        ros_distro: str,
        exec_type: str,
        ws: str,
        pkg_name: str,
        exec: str,
        env: Optional[Dict[str, str]] = None,
    ) -> None:
        """Initialize the ROS service.

        Args:
            name: Human-readable name for the service.
            id: Unique identifier for the service.
            ros_distro: ROS distribution name (e.g., 'humble', 'foxy').
            exec_type: Execution type - either 'NODE' or 'LAUNCH'.
            ws: Path to the ROS workspace.
            pkg_name: ROS package name containing the executable.
            exec: Node or launch file to execute.
            env: Optional dictionary of environment variables.

        Notes:
            If the workspace path doesn't exist or the ROS distribution is not found,
            the service state is set to FAILURE.
        """
        super().__init__(
            name,
            id,
            "",
            env=env,
            auto_start=auto_start,
            restart_on_failure=restart_on_failure,
        )

        ws = os.path.expandvars(ws)

        if not os.path.exists(ws):
            logger.error(f"Workspace {ws} does not exist. Service will be unavailable")
            self._state = ServiceState.FAILURE

        if not self.__is_ros_available(ros_distro):
            logger.error(
                "Ros does not seem to be installed. Service will be unavailable."
            )
            self._state = ServiceState.FAILURE

        self._cmd = self.__build_cmd(ros_distro, exec_type, pkg_name, ws, exec) or ""

    def __is_ros_available(self, distro: str) -> bool:
        """Check if the specified ROS distribution is installed on the system.

        Args:
            distro: Name of the ROS distribution (e.g., 'humble').

        Returns:
            True if the ROS setup directory exists, False otherwise.
        """
        return os.path.exists(f"/opt/ros/{distro}")

    def __get_source_ros_cmd(self, distro: str) -> str:
        """Get the command string to source the base ROS environment.

        Args:
            distro: ROS distribution name.

        Returns:
            A shell command string that sources the ROS environment setup script.
        """
        return f". /opt/ros/{distro}/setup.sh"

    def __build_cmd(
        self, distro: str, exec_type: str, pkg_name: str, ws: str, executable: str
    ) -> str | None:
        """Construct the full shell command to launch the ROS executable.

        Args:
            distro: ROS distribution name.
            exec_type: Type of ROS execution ('NODE' or 'LAUNCH').
            pkg_name: ROS package containing the executable.
            ws: Path to the ROS workspace.
            executable: Name of the node or launch file to execute.

        Returns:
            A full shell command string, or None if the exec_type is invalid.
        """
        exec_type = exec_type.upper()

        cmd = self.__get_source_ros_cmd(distro) + " && "
        ws_setup_file = os.path.join(ws, "install", "setup.sh")
        cmd += f". {ws_setup_file} && "

        if exec_type == "NODE":
            cmd += f"ros2 run {pkg_name} {executable}"
            return cmd
        elif exec_type == "LAUNCH":
            cmd += f"ros2 launch {pkg_name} {executable}"
            return cmd

        logger.error(
            f"{exec_type} is an unknown ROS executable type. Available types are NODE or LAUNCH"
        )
        return None
