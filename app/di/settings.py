from functools import lru_cache

from app.config.managers_settings import ManagersSettings
from app.config import APISettings, AppSettings, DeviceSettings
from app.logging import logger


@lru_cache()
def get_app_settings() -> AppSettings:
    """Get the global application settings instance.

    Uses LRU cache to ensure the settings are only created once.

    Returns:
        AppSettings: The loaded application configuration.
    """
    logger.debug("Get App settings dependency called")
    return AppSettings()


@lru_cache()
def get_device_settings() -> DeviceSettings:
    """Get the device-related settings from the application settings.

    Returns:
        DeviceSettings: Configuration specific to the device (e.g., storage paths, sensors).
    """
    logger.debug("Get device settings dependency called")
    return get_app_settings().device


@lru_cache()
def get_managers_settings() -> ManagersSettings:
    """Get the managers settings from the application settings.

    Returns:
        ManagersSettings: Configuration specific to the managers
    """
    logger.debug("Get managers settings dependency called")
    return get_app_settings().managers


@lru_cache()
def get_api_settings() -> APISettings:
    """Get the API-related settings from the application settings.

    Returns:
        APISettings: Configuration for the API layer (e.g., host, port).
    """
    logger.debug("Get API settings dependency called")
    return get_app_settings().api
