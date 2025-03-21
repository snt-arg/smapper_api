from functools import lru_cache
from app.config.config import Configuration, load_config
from app.core.service_manager import ServiceManager


@lru_cache()
def get_configuration() -> Configuration:
    print("GET CONFIGURATION CALLED")
    config = load_config("config/device_config.yaml")
    return config


@lru_cache()
def get_service_manager() -> ServiceManager:
    print("GET SERVICE MANGER CALLED")
    manager = ServiceManager()
    return manager
