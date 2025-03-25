from enum import Enum
from logging import log
import psutil
import signal
import time
import os
import subprocess
from typing import Dict, Optional, Callable


from pydantic import BaseModel
from app.logger import logger
from app.exceptions import ServiceException
from app.schemas.services import ServiceSchema


# NOTE: this can be considered a schema
class ServiceFailure(BaseModel):
    ret_code: int
    std_err: str


class ServiceState(Enum):
    """Enum representing the state of the service."""

    INACTIVE = "INACTIVE"  # Service is not running
    ACTIVE = "ACTIVE"  # Service is running
    TERMINATED = "TERMINATED"  # Service has finished cleanly
    FAILURE = "FAILURE"  # Service encountered an error


class Service:
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
    _failure_reason: Optional[ServiceFailure]
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
        self._id = id
        self._name = name
        self._cmd = cmd
        self._auto_start = auto_start
        self._restart_on_failure = restart_on_failure
        self._env = {**env, **os.environ} if env else {**os.environ}
        self._cwd = cwd or os.path.curdir

        self._state = ServiceState.INACTIVE
        self._process = None

    def start(self, restart: bool = False) -> None:
        logger.debug(f"[service:{self._id}] Starting Service")
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

        self._process = psutil.Popen(
            args=self._cmd,
            env=self._env,
            cwd=self._cwd,
            shell=True,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        self._state = ServiceState.ACTIVE

        self.poll()

    def stop(self, timeout: int = 3) -> None:
        logger.debug(f"[service:{self._id}] Stopping Service")
        self.poll()

        if (
            self._process is None
            or self._state is ServiceState.INACTIVE
            or self._state is ServiceState.FAILURE
            or self._state is ServiceState.TERMINATED
        ):
            logger.info(
                f"[service:{self._id}] Service is not ACTIVE. Ignoring request."
            )
            return

        for child_proc in self._process.children(recursive=True):
            try:
                child_proc.terminate()
                child_proc.wait(timeout)
            except psutil.TimeoutExpired:
                logger.warning(
                    f"[service:{self._id}] Timeout while stopping child process. Forcing termination."
                )
                child_proc.kill()
                child_proc.wait()

        try:
            self._process.terminate()
            self._process.wait(timeout)
        except psutil.TimeoutExpired:
            logger.warning(
                f"[service:{self._id}] Timeout while stopping process. Forcing termination."
            )
            self._process.kill()
            self._process.wait()

        self._process = None
        self._state = ServiceState.INACTIVE

    def restart(self, timeout: int = 3) -> None:
        self.stop(timeout)
        self.start()

    def get_state(self) -> ServiceState:
        self.poll()
        return self._state

    def get_schema(self) -> ServiceSchema:
        return ServiceSchema(
            name=self._name,
            id=self._id,
            cmd=self._cmd,
            cwd=self._cwd,
            env=self._env,
        )

    def _set_state(self, new_state: ServiceState) -> None:
        if new_state == self._state:
            return
        self._state = new_state
        logger.debug(
            f"[service:{self._id}] State updated: {self._state.name} -> {new_state.name}"
        )

    def poll(self) -> None:
        if self._process is None:
            return

        logger.debug(f"[service:{self._id}] Polling service process.")

        ret_code = self._process.poll()

        logger.warning(f"Process return code: {ret_code}/{self._process.is_running()}")

        if ret_code is None:
            return
        if ret_code == 0:
            logger.info(f"[service:{self._id}] Process has terminated cleanly.")
            self._state = ServiceState.TERMINATED
            return

        logger.error(f"[service:{self._id}] Process has terminated with error.")
        _, stderr = self._process.communicate()
        self._failure_reason = ServiceFailure(ret_code=ret_code, std_err=stderr)
        self._state = ServiceState.FAILURE

        # NOTE: consider a way to notify caller to Service in case of failure
        if self._restart_on_failure and self._max_restart_attempts > 0:
            self._max_restart_attempts -= 1
            logger.info(f"[service:{self._id}] Restarting service after failure.")
            self.restart()
