import os
import time
from enum import Enum
from threading import Lock, Thread
from typing import Dict, List, Optional, Tuple

from sqlalchemy.orm import Session

from app.core.exceptions import ServiceException
from app.core.services import RosbagService
from app.crud.rosbag import create_rosbag
from app.db.database import db_session
from app.logging import logger
from app.schemas.recording import (
    RecordingMetadata,
    RecordingStartRequest,
    RecordingStatus,
)
from app.schemas.ros.rosbag import RosbagMetadata, RosbagMetadataCreate
from app.utils.rosbag import (
    create_rosbag_name_with_date,
    get_rosbag_size,
    read_rosbag_metadata,
)


class RecordingState(Enum):
    IDLE = "Idle"
    INITIALIZING = "Initializing"
    RECORDING = "Recording"
    STOPPING = "Stopping"
    SAVING = "Saving"
    COMPLETED = "Completed"
    ERROR = "Error"


class RecordingManager:
    _state: RecordingState
    _service: RosbagService | None
    _start_time: float
    _end_time: float
    _elapsed_time: float
    _curr_request: Tuple[RecordingStartRequest, str] | None

    _update_thread: Thread
    _update_lock: Lock
    _update_thread_running: bool = False
    _update_thread_idle_freq: int = 5

    _db_session: Session

    _rosbag_storage_dir: str
    _ws: Optional[str]
    _env: Optional[Dict[str, str]]
    _presets: Optional[Dict[str, List[str]]]

    def __init__(
        self,
        storage_dir: str,
        ws: Optional[str],
        env: Optional[Dict[str, str]] = None,
        presets: Optional[Dict[str, List[str]]] = None,
    ):
        logger.info("Initializing Recording Manager")
        self._state = RecordingState.IDLE

        self._ws = ws
        self._env = env
        self._presets = presets

        self._create_rosbag_storage_dir(storage_dir)

        # Setup Update Thread
        self._update_thread = Thread(target=self._update_thread_cb, daemon=True)
        self._update_lock = Lock()

        self._reset_timers()

        # Start update thread
        self._update_thread_running = True
        self._update_thread.start()

    def terminate(self):
        logger.info("Terminating Recording Manager")
        if self._state is RecordingState.RECORDING:
            self.stop_recording()
        self._update_thread_running = False
        self._update_thread.join(1)

    def start_recording(self, req: RecordingStartRequest):
        if self._state is not RecordingState.IDLE:
            raise Exception("A recording is already on-going")
        self._state = RecordingState.INITIALIZING
        self._reset_timers()
        self._start_time = time.time()

        rosbag_name = create_rosbag_name_with_date(req.name, self._start_time)
        rosbag_path = os.path.join(self._rosbag_storage_dir, rosbag_name)
        self._curr_request = (req, rosbag_path)

        # Start rosbag service
        self._service = RosbagService(
            self._rosbag_storage_dir, rosbag_name, req.topics, self._ws, self._env
        )
        try:
            self._service.start()
        except ServiceException as e:
            self._state = RecordingState.ERROR
            raise e

        # Give some time to let ros2 bag create folder
        while not os.path.exists(rosbag_path):
            time.sleep(0.5)

        if not self._service.is_running():
            logger.error("Rosbag service failed to start")
            self._state = RecordingState.ERROR
            raise Exception("Failed to start rosbag service")

        self._state = RecordingState.RECORDING

    def stop_recording(self) -> RosbagMetadata:
        if self._state is not RecordingState.RECORDING:
            raise Exception("No on-going recording to stop")
        self._state = RecordingState.STOPPING
        assert self._service is not None
        self._service.stop()
        self._end_time = time.time()

        self._state = RecordingState.SAVING

        rosbag = self._save_to_db()

        self._state = RecordingState.COMPLETED

        self._reset_timers()
        self._state = RecordingState.IDLE

        return rosbag

    def get_presets(self) -> Dict[str, List[str]]:
        return self._presets if self._presets else {}

    def get_status(self) -> RecordingStatus:
        if self._state is RecordingState.RECORDING and self._curr_request:
            metadata, rosbag_path = self._curr_request

            return RecordingStatus(
                state=self._state.value,
                metadata=RecordingMetadata(
                    bag_name=metadata.name,
                    start_time=self._start_time,
                    elapsed_time=self.get_elapsed_time(),
                    bag_size=get_rosbag_size(rosbag_path),
                    topics=metadata.topics,
                ),
            )
        return RecordingStatus(state=self._state.value)

    def get_state(self) -> RecordingState:
        return self._state

    def get_elapsed_time(self) -> float:
        self._update_lock.acquire()
        elapsed = self._elapsed_time
        self._update_lock.release()
        return elapsed

    def _update_thread_cb(self):
        while self._update_thread_running:
            now = time.time()
            self._elapsed_time = now - self._start_time

            if self._state is RecordingState.RECORDING:
                time.sleep(1)
            else:
                time.sleep(self._update_thread_idle_freq)

    def _reset_timers(self):
        self._start_time = 0
        self._end_time = 0
        self._elapsed_time = 0

    def _create_rosbag_storage_dir(self, path: str):
        path = os.path.expandvars(path)
        self._rosbag_storage_dir = path
        if not os.path.exists(path):
            logger.warning(
                f"Rosbags storage folder {path} does not exist. Creating it."
            )
            os.makedirs(path)

    def _save_to_db(self) -> RosbagMetadata:
        if self._curr_request is None:
            raise Exception("No recording to save.")

        metadata, rosbag_path = self._curr_request
        rosbag_metadata = read_rosbag_metadata(rosbag_path)

        tags = ",".join(set(metadata.tags)) if metadata.tags else ""

        rosbag = RosbagMetadataCreate(
            name=metadata.name,
            size=get_rosbag_size(rosbag_path),
            duration=self._end_time - self._start_time,
            start_time=self._start_time,
            end_time=self._end_time,
            detail=metadata.detail,
            tags=tags,
            rosbag_path=rosbag_path,
            topics=rosbag_metadata.topics,
        )

        with db_session() as db:
            rosbag_model = create_rosbag(db, rosbag)
            rosbag_schema = RosbagMetadata.model_validate(rosbag_model)

        return rosbag_schema
