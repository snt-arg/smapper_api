from functools import lru_cache
from typing import Annotated, List
from fastapi import APIRouter, Depends, HTTPException
from app.config.settings import DeviceSettings
from app.core.service_manager import (
    ServiceManager,
    ServiceManagerException,
)
from app.core.services.service import ServiceState
from app.dependencies import get_device_settings, get_service_manager
from app.schemas.services import RosServiceSchema, ServiceSchema, ServiceStateSchema
from app.exceptions import NotYetImplemented


router = APIRouter(prefix="/api/v1")


@router.get("/services", description="Get a list of available services")
def get_services(
    config: Annotated[DeviceSettings, Depends(get_device_settings)]
) -> List[ServiceSchema | RosServiceSchema]:
    return config.services


@router.get("/services/{id}", description="Get current state of service with id")
def get_service_by_id(
    id: str,
    manager: Annotated[ServiceManager, Depends(get_service_manager)],
) -> ServiceSchema:
    raise NotYetImplemented("Endpoint /services/{id} has not yet been implemented")


@router.get("/services/{id}/state", description="Get current state of service with id")
def get_service_state_by_id(
    id: str,
    manager: Annotated[ServiceManager, Depends(get_service_manager)],
) -> ServiceStateSchema:
    state = manager.get_service_state(id)
    return ServiceStateSchema(state=state.name, value=state.value)


@router.post("/services/{id}/start", description="Start service with id")
def start_service_with_id(
    id: str,
    manager: Annotated[ServiceManager, Depends(get_service_manager)],
):
    manager.start_service(id)
    # TODO: return service state once it is implemented in service_manager
    return {}


@router.post("/services/{id}/stop", description="Stop service with id")
def stop_service_with_id(
    id: str,
    manager: Annotated[ServiceManager, Depends(get_service_manager)],
):
    manager.stop_service(id)
    # TODO: return service state once it is implemented in service_manager
    return {}
