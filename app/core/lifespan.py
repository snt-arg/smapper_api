from contextlib import asynccontextmanager

from fastapi import FastAPI
from app.config.managers_settings import ManagersSettings
from app.core.managers import ServiceManager
from app.logging import logger
from app.di import (
    get_recording_manager,
    get_topic_monitor_runner,
    get_service_manager,
    get_app_settings,
)


def init_services(
    service_manager: ServiceManager,
    manager_settings: ManagersSettings,
) -> None:
    """Initialize and register services on application startup.

    Args:
        service_manager: The service manager instance used to register services.
        config: The loaded device configuration, which includes the list of services to register.
    """
    for service in manager_settings.services:
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
    settings = get_app_settings()

    service_manager = get_service_manager()
    recording_manager = get_recording_manager()
    topic_monitor_runner = get_topic_monitor_runner()

    init_services(service_manager, settings.managers)
    if topic_monitor_runner:
        topic_monitor_runner.start()

    yield
    logger.info("Terminating Application")

    # Executed on shutdown
    service_manager.terminate()
    recording_manager.terminate()

    if topic_monitor_runner:
        topic_monitor_runner.stop()
