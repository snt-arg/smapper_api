from contextlib import asynccontextmanager
from typing import Annotated

from fastapi import Depends, FastAPI
from app.core.service_manager import ServiceManager
from app.config.settings import DeviceSettings
from app.schemas import ServiceSchema, RosServiceSchema
from app.logger import logger
from app.dependencies import (
    get_device_settings,
    get_service_manager,
)


def init_services(
    service_manager: Annotated[ServiceManager, Depends(get_service_manager)],
    config: Annotated[DeviceSettings, Depends(get_device_settings)],
) -> None:
    logger.info("Setting up services")
    for service in config.services:
        if isinstance(service, ServiceSchema):
            service_manager.add_service(service.id, service.name, cmd=service.cmd)
        elif isinstance(service, RosServiceSchema):
            service_manager.add_service(service.id, service.name, cmd=service.exec)


def terminate_services(
    service_manager: Annotated[ServiceManager, Depends(get_service_manager)],
) -> None:
    service_manager.stop_all()


@asynccontextmanager
async def lifespan(
    app: FastAPI,
):
    # Executed on startup
    service_manager = get_service_manager()
    config = get_device_settings()

    init_services(service_manager, config)

    yield
    # Executed on shutdown

    terminate_services(service_manager)
