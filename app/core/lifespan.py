from contextlib import asynccontextmanager
from typing import Annotated
from typing_extensions import runtime

from fastapi import Depends, FastAPI
from app.core.managers import ServiceManager
from app.config.settings import DeviceSettings
from app.logging import logger
from app.di import (
    get_device_settings,
    get_recording_manager,
    get_topic_monitor_runner,
    get_service_manager,
)


def init_services(
    service_manager: Annotated[ServiceManager, Depends(get_service_manager)],
    config: Annotated[DeviceSettings, Depends(get_device_settings)],
) -> None:
    """Initialize and register services on application startup.

    Args:
        service_manager: The service manager instance used to register services.
        config: The loaded device configuration, which includes the list of services to register.
    """
    for service in config.services:
        service_manager.add_service(service)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """FastAPI lifespan context to handle startup and shutdown events.

    On startup:
        - Initializes and registers services.
        - Starts the topic monitor.

    On shutdown:
        - Stops all running services.
        - Stops the topic monitor runner.

    Args:
        app: The FastAPI application instance.

    Yields:
        Control back to FastAPI's event loop during application runtime.
    """
    logger.info("Initializing Application")
    # Executed on startup
    service_manager = get_service_manager()
    recording_manager = get_recording_manager()
    topic_monitor_runner = get_topic_monitor_runner()
    config = get_device_settings()

    init_services(service_manager, config)
    if topic_monitor_runner:
        topic_monitor_runner.start()

    yield
    logger.info("Terminating Application")

    # Executed on shutdown
    service_manager.terminate()
    recording_manager.terminate()

    if topic_monitor_runner:
        topic_monitor_runner.stop()
