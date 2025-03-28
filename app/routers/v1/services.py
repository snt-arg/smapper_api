from typing import Annotated, List
from fastapi import APIRouter, Depends
from app.config.settings import DeviceSettings
from app.core.service_manager import (
    ServiceManager,
)
from app.dependencies import get_device_settings, get_service_manager
from app.schemas.services import RosServiceSchema, ServiceSchema, ServiceStateSchema


router = APIRouter(prefix="/api/v1")


@router.get(
    "/services",
    description="Get a list of all configured services, including both standard and ROS-based services.",
)
def get_services(
    config: Annotated[DeviceSettings, Depends(get_device_settings)],
) -> List[ServiceSchema | RosServiceSchema]:
    return config.services


@router.get(
    "/services/{id}",
    description="Retrieve the full configuration of a specific service by its ID.",
)
def get_service_by_id(
    id: str,
    manager: Annotated[ServiceManager, Depends(get_service_manager)],
) -> ServiceSchema:
    return manager.get_service_by_id(id)


@router.get(
    "/services/{id}/state",
    description="Get the current runtime state (e.g., ACTIVE, INACTIVE, FAILURE) of a specific service.",
)
def get_service_state_by_id(
    id: str,
    manager: Annotated[ServiceManager, Depends(get_service_manager)],
) -> ServiceStateSchema:
    state = manager.get_service_state(id)
    return ServiceStateSchema(state=state.name)


@router.post(
    "/services/{id}/start",
    description="Start the specified service by its ID. Returns the updated state after starting.",
)
def start_service_with_id(
    id: str,
    manager: Annotated[ServiceManager, Depends(get_service_manager)],
) -> ServiceStateSchema:
    manager.start_service(id)
    state = manager.get_service_state(id)
    return ServiceStateSchema(state=state.name)


@router.post(
    "/services/{id}/stop",
    description="Stop the specified service by its ID. Returns the updated state after stopping.",
)
def stop_service_with_id(
    id: str,
    manager: Annotated[ServiceManager, Depends(get_service_manager)],
) -> ServiceStateSchema:
    manager.stop_service(id)
    state = manager.get_service_state(id)
    return ServiceStateSchema(state=state.name)
