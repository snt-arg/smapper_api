from typing import Annotated
from fastapi import APIRouter, Depends
from app.core.recording_manager import RecordingManager
from app.dependencies import get_recording_manager
from app.schemas.recording import RecordingStatus, RecordingStartRequest
from app.schemas.ros.rosbag import RosbagMetadata


router = APIRouter(prefix="/api/v1/recording")


@router.post(
    "/start",
    description="Start a new rosbag recording session.",
    response_model=RecordingStatus,
)
def start_recording(
    request: RecordingStartRequest,
    manager: Annotated[RecordingManager, Depends(get_recording_manager)],
):
    manager.start_recording(request)
    return manager.get_status()


@router.post(
    "/stop",
    description="Stop the current running rosbag recording session.",
    response_model=RosbagMetadata,
)
def stop_recording(
    manager: Annotated[RecordingManager, Depends(get_recording_manager)],
):
    return manager.stop_recording()


@router.get(
    "/",
    description="Get the current status of the rosbag recording session.",
    response_model=RecordingStatus,
)
def get_bag_recording_state(
    manager: Annotated[RecordingManager, Depends(get_recording_manager)],
):
    return manager.get_status()
