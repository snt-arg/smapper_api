from .service import ServiceState, Service
import os
from app.logger import logger


class RosService(Service):
    def __init__(
        self,
        name: str,
        id: str,
        distro: str,
        exec_type: str,
        ws: str,
        pkg_name: str,
        executable: str,
        env: dict[str, str] | None = None,
    ) -> None:
        super().__init__(name, id, "", env)

        if not os.path.exists(ws):
            logger.error(f"Workspace {ws} does not exist. Service will be unavailable")
            self._state = ServiceState.ERROR

        if not self.__is_ros_available(distro):
            logger.error(
                "Ros does not seem to be installed. Service will be unavailable."
            )
            self._state = ServiceState.ERROR

        self._cmd = self.__build_cmd(distro, exec_type, pkg_name, executable)

    def __is_ros_available(self, distro: str) -> bool:
        return os.path.exists(f"/opt/ros/{distro}")

    def __get_source_ros_cmd(self, distro: str) -> str:
        return f"source /opt/ros/{distro}/setup.bash"

    def __build_cmd(
        self, distro: str, exec_type: str, pkg_name: str, executable: str
    ) -> str | None:
        exec_type = exec_type.upper()

        cmd = self.__get_source_ros_cmd(distro) + "&&"

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
