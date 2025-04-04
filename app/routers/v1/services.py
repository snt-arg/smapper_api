from typing import Annotated, List
from fastapi import APIRouter, Depends
from app.settings import DeviceSettings
from app.core.service_manager import (
    ServiceManager,
)
from app.dependencies import get_device_settings, get_service_manager
from app.schemas.services import (
    RosServiceConfigSchema,
    ServiceConfigSchema,
    ServiceSchema,
)


router = APIRouter(prefix="/api/v1")


@router.get(
    "/services",
    description="Get a list of all configured services, including both standard and ROS-based services.",
)
def get_services(
    manager: Annotated[ServiceManager, Depends(get_service_manager)],
) -> List[ServiceSchema]:
    return manager.get_services()


@router.get(
    "/services/{id}",
    description="Retrieve the full configuration of a specific service by its ID.",
)
def get_service_by_id(
    id: str,
    manager: Annotated[ServiceManager, Depends(get_service_manager)],
) -> ServiceSchema:
    return manager.get_service_by_id(id)


@router.post(
    "/services/{id}/start",
    description="Start the specified service by its ID. Returns the updated state after starting.",
)
def start_service_with_id(
    id: str,
    manager: Annotated[ServiceManager, Depends(get_service_manager)],
) -> ServiceSchema:
    manager.start_service(id)
    return manager.get_service_by_id(id)


@router.post(
    "/services/{id}/stop",
    description="Stop the specified service by its ID. Returns the updated state after stopping.",
)
def stop_service_with_id(
    id: str,
    manager: Annotated[ServiceManager, Depends(get_service_manager)],
) -> ServiceSchema:
    manager.stop_service(id)
    return manager.get_service_by_id(id)
