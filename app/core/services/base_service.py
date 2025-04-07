from enum import Enum
import signal
import psutil
import os
import subprocess
from typing import Dict, Optional


from app.logging import logger
from app.core.exceptions import ServiceException
from app.schemas.service import ServiceFailure, ServiceStatus


class ServiceState(Enum):
    """Enum representing the state of the service.

    Members:
        INACTIVE: Service is not running.
        ACTIVE: Service is currently running.
        TERMINATED: Service completed successfully and exited.
        FAILURE: Service encountered an error and failed.
    """

    INACTIVE = "Inactive"
    ACTIVE = "Active"
    TERMINATED = "Terminated"
    FAILURE = "Failure"


class Service:
    """Base class for managing long-lived external processes (services)
    with lifecycle control.
    """

    # Config
    _id: str
    _name: str
    _restart_on_failure: bool
    _auto_start: bool
    _cmd: str
    _cwd: Optional[str]
    _env: Optional[Dict[str, str]]

    _process: Optional[psutil.Popen]
    _state: ServiceState
    _failure_reason: Optional[ServiceFailure] = None
    # TODO: expose this to a service setting
    _max_restart_attempts: int = 5

    def __init__(
        self,
        id: str,
        name: str,
        cmd: str,
        auto_start: bool = False,
        restart_on_failure: bool = False,
        env: Optional[Dict[str, str]] = None,
        cwd: Optional[str] = None,
    ) -> None:
        """Initialize the Service with its command, environment, and lifecycle behavior.

        Args:
            id: Unique identifier for the service.
            name: Human-readable name for the service.
            cmd: The command to execute the service.
            auto_start: Whether to automatically start the service on initialization.
            restart_on_failure: Whether to restart the service if it crashes.
            env: Optional environment variables to override the default.
            cwd: Optional working directory from which to launch the service.
        """
        self._id = id
        self._name = name
        self._cmd = cmd
        self._auto_start = auto_start
        self._restart_on_failure = restart_on_failure
        self._env = {**env, **os.environ} if env else {**os.environ}
        self._cwd = os.path.expandvars(cwd) if cwd else os.path.curdir

        self._state = ServiceState.INACTIVE
        self._process = None

        if self._auto_start:
            self.start()

    def start(self, restart: bool = False) -> None:
        """Start the service process.

        If already running and restart is False, raises a ServiceException.
        Handles failure cases and updates internal state.
        """
        self.poll()
        if (
            self._process
            and self._process.is_running()
            and self._state is ServiceState.ACTIVE
            and not restart
        ):
            # FIX: Modify ServiceException to have cmd, stderr as optional, possibly removing cmd completly
            raise ServiceException(
                self._id,
                "Service is already running and restart was not specified.",
                self._cmd,
                "",
            )
        logger.info(f"[service:{self._id}] Create service process")

        try:
            self._process = psutil.Popen(
                args=self._cmd,
                env=self._env,
                cwd=self._cwd,
                shell=True,
                text=True,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
        except Exception as e:
            logger.error(f"[service:{self._id}] Failed to start service: {e}")
            self._set_state(ServiceState.FAILURE)
            self._failure_reason = ServiceFailure(ret_code=-1, std_err=str(e))
            raise ServiceException(
                self._id,
                f"[service:{self._id}] Failed to start service",
                self._cmd,
                str(e),
            )

        self._state = ServiceState.ACTIVE

    def stop(self, timeout: int = 4) -> None:
        """Stop the service process and clean up child processes.

        Args:
            timeout: Maximum time (in seconds) to wait before force-killing.
        """
        logger.debug(f"[service:{self._id}] Stopping Service")
        self.poll()

        if self._process is None or self._state is not ServiceState.ACTIVE:
            logger.info(
                f"[service:{self._id}] Service is not ACTIVE. Ignoring request."
            )
            return

        self._terminate_child_processes(timeout)

        try:
            if self._process.is_running():
                self._process.terminate()
                self._process.wait(timeout)
        except psutil.NoSuchProcess:
            pass
        except psutil.TimeoutExpired:
            logger.warning(
                f"[service:{self._id}] Timeout while stopping process. Forcing termination."
            )
            self._process.kill()
            self._process.wait()

        self._state = ServiceState.INACTIVE

    def restart(self, timeout: int = 3) -> None:
        """Restart the service process by stopping and starting it again.

        Args:
            timeout: Timeout to wait while stopping before restarting.
        """
        self.stop(timeout)
        self.start()

    def get_state(self) -> ServiceState:
        """Return the current state of the service."""
        return self._state

    def get_status(self) -> ServiceStatus:
        return ServiceStatus(
            id=self._id,
            name=self._name,
            state=self.get_state().value,
            failure=self._failure_reason,
        )

    def is_running(self) -> bool:
        """Check whether the service is currently running and marked ACTIVE."""
        return self._state is ServiceState.ACTIVE

    def _set_state(self, new_state: ServiceState) -> None:
        """Update the service's internal state, with optional logging.

        Args:
            new_state: The new ServiceState to apply.
        """
        if new_state == self._state:
            return
        self._state = new_state
        logger.debug(
            f"[service:{self._id}] State updated: {self._state.name} -> {new_state.name}"
        )

    def poll(self) -> None:
        """Check the current status of the service process.

        Updates the internal state depending on exit codes.
        Handles automatic restarts if enabled.
        """
        if self._process is None:
            return

        logger.debug(f"[service:{self._id}] Polling service process.")

        ret_code = self._process.poll()

        if ret_code is None:
            return

        if ret_code == 0 and self._state is not ServiceState.TERMINATED:
            logger.info(f"[service:{self._id}] Process has terminated cleanly.")
            self._state = ServiceState.TERMINATED
            self._process = None
            return

        if self._state is not ServiceState.FAILURE:
            _, stderr = self._process.communicate()
            self._failure_reason = ServiceFailure(ret_code=ret_code, std_err=stderr)
            self._state = ServiceState.FAILURE
            self._process = None
            logger.error(
                f"[service:{self._id}] Process has terminated with error -> {self._failure_reason.__repr__()}"
            )

        # NOTE: consider a way to notify caller to Service in case of failure
        if self._restart_on_failure and self._max_restart_attempts > 0:
            self._max_restart_attempts -= 1
            logger.info(f"[service:{self._id}] Restarting service after failure.")
            self.restart()

    def _terminate_child_processes(self, timeout=4):
        """
        Gracefully terminate all child processes with improved error handling.

        Args:
            timeout (float, optional): Timeout for graceful shutdown. Defaults to 4 seconds.
        """
        assert self._process is not None
        # Get children processes only once to avoid potential race conditions
        child_processes = self._process.children(recursive=True)

        if not child_processes:
            return

        # Attempt a graceful shutdown with SIGINT
        for child_proc in child_processes:
            try:
                if child_proc.is_running():
                    child_proc.send_signal(signal.SIGINT)
            except psutil.NoSuchProcess:
                continue

        # Wait for processes to terminate
        for child_proc in child_processes:
            try:
                if child_proc.is_running():
                    child_proc.wait(timeout)
            except psutil.NoSuchProcess:
                continue
            except psutil.TimeoutExpired:
                try:
                    logger.warning(
                        f"[service:{self._id}] Timeout while stopping child process. Forcing termination."
                    )
                    child_proc.kill()
                    child_proc.wait(timeout=2)
                except (psutil.NoSuchProcess, psutil.TimeoutExpired):
                    logger.error(
                        f"[service:{self._id}] Unable to terminate child process {child_proc.pid}"
                    )
