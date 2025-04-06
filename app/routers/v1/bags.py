from typing import Annotated, List
from fastapi import APIRouter, Depends, HTTPException
from app.core.bag_manager import BagRecordingManager
from app.dependencies import get_bag_manager
from app.exceptions import BagNotFoundException
from app.schemas import BagSchema, BagRecordingRequestSchema, BagCreationResponse
from app.schemas.bags import RecordingStatus


router = APIRouter(prefix="/api/v1")


@router.get(
    "/bags",
    description="Return a list of all available bag recordings with associated metadata.",
)
def get_bags(
    bag_manager: Annotated[BagRecordingManager, Depends(get_bag_manager)],
) -> List[BagSchema]:
    return bag_manager.get_bags()


@router.get(
    "/bags/{id}",
    description="Fetch detailed information about a specific bag by its ID.",
)
def get_bag_by_id(
    id: str, bag_manager: Annotated[BagRecordingManager, Depends(get_bag_manager)]
) -> BagSchema:
    try:
        bag = bag_manager.get_bag_by_id(id)
    except BagNotFoundException as e:
        raise HTTPException(404, e.detail)

    return bag


@router.post(
    "/bags/record/start",
    description="Start a new rosbag recording session for the selected topics.",
)
def create_bag(
    request: BagRecordingRequestSchema,
    bag_manager: Annotated[BagRecordingManager, Depends(get_bag_manager)],
) -> BagCreationResponse:
    return bag_manager.create_bag(request)


@router.post(
    "/bags/record/stop",
    description="Stop the currently running rosbag recording session.",
)
def stop_bag_recording(
    bag_manager: Annotated[BagRecordingManager, Depends(get_bag_manager)],
) -> BagSchema:
    try:
        bag_schema = bag_manager.stop_bag_recording()
    except Exception as e:
        raise HTTPException(status_code=404, detail=f"{e}")

    return bag_schema


@router.get(
    "/bags/record/state",
    description="Get the current state of the rosbag recording service.",
)
def get_bag_recording_state(
    bag_manager: Annotated[BagRecordingManager, Depends(get_bag_manager)],
) -> RecordingStatus:
    state = bag_manager.get_recording_state()
    return RecordingStatus(status=state.name)
