from functools import lru_cache

from pydantic import ValidationError
from app.config.settings import APISettings, AppSettings, DeviceSettings
from app.core.service_manager import ServiceManager
from app.logger import logger


@lru_cache()
def get_app_settings() -> AppSettings:
    return AppSettings()


@lru_cache()
def get_device_settings() -> DeviceSettings:
    return get_app_settings().device


@lru_cache()
def get_api_settings() -> APISettings:
    return get_app_settings().api


@lru_cache()
def get_service_manager() -> ServiceManager:
    logger.debug("Get service manager dependency called")
    manager = ServiceManager()
    return manager
