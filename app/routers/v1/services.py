from functools import lru_cache
from typing import Annotated
from fastapi import APIRouter, Depends, HTTPException
from app.config.config import Configuration
from app.core.service_manager import (
    ServiceManager,
    ServiceManagerException,
    get_service_manager,
)
from app.core.services.service import ServiceException, ServiceState


router = APIRouter(prefix="/api/v1")


@lru_cache()
def get_configuration():
    return Configuration.load("app/config/device.yaml")


@router.get("/services", description="Get a list of available services")
def get_sensors(config: Annotated[Configuration, Depends(get_configuration)]):
    return config.services


@router.get("/services/{id}", description="Get current state of service with id")
def get_sensor(
    id: str,
    manager: Annotated[ServiceManager, Depends(get_service_manager)],
) -> ServiceState:
    return manager.get_service_state(id)


@router.post("/services/{id}/start", description="Start service with id")
def start_sensor(
    id: str,
    manager: Annotated[ServiceManager, Depends(get_service_manager)],
):
    return manager.start_service(id)


@router.post("/services/{id}/stop", description="Stop service with id")
def stop_sensor(
    id: str,
    manager: Annotated[ServiceManager, Depends(get_service_manager)],
):
    return manager.stop_service(id)
