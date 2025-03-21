from functools import lru_cache
from app.config.config import Configuration, load_config
from app.core.service_manager import ServiceManager
from app.logger import logger


@lru_cache()
def get_configuration() -> Configuration:
    logger.debug("Get configuration dependency called")
    config = load_config("config/device_config.yaml")
    return config


@lru_cache()
def get_service_manager() -> ServiceManager:
    logger.debug("Get service manager dependency called")
    manager = ServiceManager()
    return manager
