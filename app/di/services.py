from functools import lru_cache

from app.core.service_manager import ServiceManager
from app.logging import logger


@lru_cache()
def get_service_manager() -> ServiceManager:
    """Get a singleton instance of the ServiceManager.

    Returns:
        ServiceManager: Manages the lifecycle of background services.
    """
    logger.debug("Get service manager dependency called")
    manager = ServiceManager()
    return manager
