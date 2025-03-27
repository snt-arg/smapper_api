from typing import Dict, Optional
from .service import ServiceState, Service
import os
from app.logger import logger


class RosService(Service):
    def __init__(
        self,
        name: str,
        id: str,
        ros_distro: str,
        exec_type: str,
        ws: str,
        pkg_name: str,
        exec: str,
        env: Optional[Dict[str, str]] = None,
    ) -> None:
        super().__init__(name, id, "", env=env)

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
        return os.path.exists(f"/opt/ros/{distro}")

    def __get_source_ros_cmd(self, distro: str) -> str:
        return f". /opt/ros/{distro}/setup.sh"

    def __build_cmd(
        self, distro: str, exec_type: str, pkg_name: str, ws: str, executable: str
    ) -> str | None:
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
