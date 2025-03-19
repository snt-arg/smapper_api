from enum import Enum
import os
import subprocess
from typing import Optional

from .logger import get_logger


class ProcessState(Enum):
    INACTIVE = 0
    ACTIVE = 1
    ERROR = 2


class ProcessHandler:
    def __init__(
        self, name: str, cmd: str | list[str], env: dict[str, str] | None = None
    ) -> None:
        self.name = name
        self.cmd = self.__preprocess_cmd(cmd)
        self.env = env or {}
        self._process: Optional[subprocess.Popen] = None
        self._logger = get_logger(name)
        self._sigint_timeout = 2
        self._state = ProcessState.INACTIVE

    def start(self) -> bool:
        self.__update_state()
        if self.is_running():
            self._logger.warning("Process is already running.")
            return True

        self._logger.info(f"Starting Process")
        full_env = {**os.environ, **self.env}
        try:
            self._process = subprocess.Popen(self.cmd, env=full_env)
            self._state = ProcessState.ACTIVE
        except Exception as e:
            self._logger.error(f"Failed to start process with error: {e}")
            self._state = ProcessState.ERROR
            return False

        return True

    def stop(self) -> bool:
        self.__update_state()
        if self._state == ProcessState.ERROR:
            self._logger.warning("Process was not able to start.")
            return False

        if not self.is_running() or self._process is None:
            self._logger.warning("Process is already INACTIVE.")
            return False

        try:
            self._logger.info(
                f"Sending SIGINT signal. Waiting maximum of {self._sigint_timeout} seconds"
            )
            self._process.terminate()
            self._process.wait(self._sigint_timeout)
        except subprocess.TimeoutExpired:
            self._logger.warning(
                f"Timeout. Forcing process to terminate. Sending SIGKILL"
            )
            self._process.kill()
            self._process.wait()
        finally:
            self._process = None
            self._state = ProcessState.INACTIVE
            return True

    def is_running(self) -> bool:
        self.__update_state()
        return self._state == ProcessState.ACTIVE

    def get_state(self) -> ProcessState:
        self.__update_state()
        return self._state

    def __update_state(self):
        if self._state == ProcessState.ERROR:
            return
        if self._process is None or self._process.poll() is not None:
            # NOTE: we could make use of returncode in the future
            # self.returncode = self._process.poll() if self._process else None
            self._state = ProcessState.INACTIVE
            return
        self._state = ProcessState.ACTIVE

    def __preprocess_cmd(self, cmd: str | list[str]) -> list[str]:
        if isinstance(cmd, str):
            assert cmd != "", "A Command cannot be an empty string"
            return cmd.split(" ")
        assert len(cmd) > 1, "A command cannot be an emtpy list"
        return cmd
