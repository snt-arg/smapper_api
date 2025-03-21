from enum import Enum
import time
import os
import subprocess
from typing import Optional, Callable
import logging

# TODO: Logging is not working properly
logger = logging.getLogger("smapper_api")


# Custom Exception Classes
class ServiceException(Exception):
    """Exception raised when a service fails."""

    pass


class ServiceState(Enum):
    """Enum representing the state of the service."""

    INACTIVE = 0  # Service is not running
    ACTIVE = 1  # Service is running
    ERROR = 2  # Service encountered an error
    STARTING = 3  # Service is in the process of starting
    STOPPING = 4  # Service is in the process of stopping
    RESTARTING = 5  # Service is in the process of restarting


class Service:
    def __init__(
        self,
        name: str,
        id: str,
        cmd: str,
        env: Optional[dict[str, str]] = None,
        on_state_change: Optional[Callable[[ServiceState], None]] = None,
    ) -> None:
        """
        Initializes the service with a name, id and command to run, and optional environment variables.

        Args:
            name (str): The name of the service.
            id (str): A unique identifier
            cmd (str): The command to run the service. Command is executd with shell
            env (Optional[dict], optional): A dictionary of environment variables to set for the service. Defaults to None.
            on_state_change (Optional[Callable[[ServiceState], None]]): A hook which checking if state is changed.
        """
        self.name = name
        self.id = id
        self._cmd = cmd
        self._env = env or {}
        self._process: Optional[subprocess.Popen] = None
        self._returncode: Optional[int] = None
        self._state = ServiceState.INACTIVE
        self._on_state_change = on_state_change

    def _set_state(self, new_state: ServiceState) -> None:
        if self._state != new_state:
            logger.debug(f"Changing state: {self._state} -> {new_state.name}")
            self._state = new_state
            if self._on_state_change:
                self._on_state_change(self._state)

    def start(self) -> None:
        """
        Starts the service by executing the command. If the service is already in the ERROR state,
        raises a ServiceStartException. Checks the process state to confirm the service starts successfully.

        Raises:
            ServiceException: If the service fails to start, or if the service is in an ERROR state.

        Sets the state to:
            - `STARTING` when the service is being started.
            - `ACTIVE` if the service starts successfully.
            - `ERROR` if there is any failure.
        """
        if self._state == ServiceState.ERROR:
            raise ServiceException(f"Service '{self.name}' is unavailable.")

        self._set_state(ServiceState.STARTING)

        try:
            logger.info(f"Starting service: {self.id}:{self.name}")
            # Set up environment variables for the subprocess
            full_env = {**os.environ, **self._env}
            self._process = subprocess.Popen(
                self._cmd,
                env=full_env,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            # Wait a short time to allow the process to initialize
            time.sleep(0.1)

            self._set_state(ServiceState.ACTIVE)
        except Exception as e:
            logger.error(f"Failed to start process with error: {e}")
            self._set_state(ServiceState.ERROR)
            raise ServiceException(f"Failed to start service '{self.name}'.")

        # Check if the process is running
        retcode = self._process.poll()
        if retcode is not None and retcode > 0:
            logger.error(f"Failed to start process for service '{self.name}'.")
            self._set_state(ServiceState.ERROR)
            raise ServiceException(f"Failed to start service '{self.name}'.")

    def stop(self, timeout=2) -> None:
        """
        Stops the service by terminating its process. If the service is not running, logs a warning and does nothing.

        Raises:
            ServiceException: If the service fails to stop, or if the service is in an ERROR state.

        Sets the state to:
            - `STOPPING` when the service is being stopped.
            - `INACTIVE` when the service is stopped successfully.
            - `ERROR` if stopping the service encounters an error.
        """
        if self._state == ServiceState.ERROR:
            raise ServiceException(f"Service '{self.name}' is unavailable.")

        if not self.is_running() or self._process is None:
            logger.warning(f"Service '{self.name}' is not running.")
            return

        self._set_state(ServiceState.STOPPING)

        try:
            logger.info(f"Stopping service {self.id}:{self.name}")
            self._process.terminate()
            self._process.wait(timeout)
        except subprocess.TimeoutExpired:
            logger.warning(
                f"Timeout while stopping '{self.name}'. Forcing termination."
            )
            self._process.kill()
            self._process.wait()
        except Exception as e:
            logger.error(f"Failed to stop process: {e}")
            raise ServiceException(f"Failed to stop service '{self.name}'.")

        self._process = None
        self._set_state(ServiceState.INACTIVE)

    def restart(self) -> None:
        """
        Restarts the service by terminating its process and starting it automatically.

        Raises:
            ServiceException: If the service fails to restart, or if the service is in an ERROR state.

        Sets the state to:
            - `RESTARTING` when the service is stopped successfully.
        """
        if self._state == ServiceState.ERROR:
            raise ServiceException(f"Service '{self.name}' is unavailable.")

        self._set_state(ServiceState.RESTARTING)

        try:
            self.stop()
            self.start()
        except Exception as e:
            logger.error(f"Failed to restart service: {e}")
            raise ServiceException(f"Failed to restart service '{self.name}'.")

    def is_running(self) -> bool:
        return self._state == ServiceState.ACTIVE

    def get_state(self) -> ServiceState:
        self._poll()
        return self._state

    def get_returncode(self) -> Optional[int]:
        """
        Returns the return code of the service process if it has finished, otherwise None.

        Returns:
            Optional[int]: The return code of the service process or None if still running.
        """
        if self._process:
            return self._process.returncode
        return self._returncode

    def _poll(self) -> None:
        if self._state == ServiceState.ERROR:
            return
        if self._process is None or self._process.poll() is not None:
            self._set_state(ServiceState.INACTIVE)
        else:
            self._set_state(ServiceState.ACTIVE)
