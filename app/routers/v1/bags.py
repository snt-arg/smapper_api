from os import stat
from typing import Annotated, List
from fastapi import APIRouter, Depends, HTTPException
from app.core.bag_manager import BagManager
from app.dependencies import get_bag_manager
from app.exceptions import BagNotFoundException
from app.schemas import BagSchema, BagRecordingRequestSchema, BagCreationResponse
from app.schemas.services import ServiceStateSchema


router = APIRouter(prefix="/api/v1")


@router.get("/bags")
def get_bags(
    bag_manager: Annotated[BagManager, Depends(get_bag_manager)],
) -> List[BagSchema]:
    return bag_manager.get_bags()


@router.get("/bags/{id}")
def get_bag_by_id(
    id: str, bag_manager: Annotated[BagManager, Depends(get_bag_manager)]
) -> BagSchema:
    try:
        bag = bag_manager.get_bag_by_id(id)
    except BagNotFoundException as e:
        raise HTTPException(404, e.detail)

    return bag


@router.post("/bags/record/start")
def create_bag(
    request: BagRecordingRequestSchema,
    bag_manager: Annotated[BagManager, Depends(get_bag_manager)],
) -> BagCreationResponse:
    return bag_manager.create_bag(request)


@router.post("/bags/record/stop")
def stop_bag_recording(
    bag_manager: Annotated[BagManager, Depends(get_bag_manager)],
) -> BagSchema:
    try:
        bag_schema = bag_manager.stop_bag_recording()
    except Exception as e:
        raise HTTPException(status_code=404, detail=f"{e}")

    return bag_schema


@router.get("/bags/record/state")
def get_bag_recording_state(
    bag_manager: Annotated[BagManager, Depends(get_bag_manager)],
) -> ServiceStateSchema:
    state = bag_manager.get_recording_state()
    return ServiceStateSchema(state=state.name, value=state.value)
