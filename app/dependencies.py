from functools import lru_cache

from app.settings import APISettings, AppSettings, DeviceSettings
from app.core.ros.ros_factory import create_topic_monitor_runner
from app.core.service_manager import ServiceManager
from app.core.bag_manager import BagRecordingManager
from app.logger import logger
from typing import TYPE_CHECKING, Optional


if TYPE_CHECKING:
    from app.core.ros.topic_monitor import TopicMonitorRunner


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
def get_api_settings() -> APISettings:
    """Get the API-related settings from the application settings.

    Returns:
        APISettings: Configuration for the API layer (e.g., host, port).
    """
    logger.debug("Get API settings dependency called")
    return get_app_settings().api


@lru_cache()
def get_service_manager() -> ServiceManager:
    """Get a singleton instance of the ServiceManager.

    Returns:
        ServiceManager: Manages the lifecycle of background services.
    """
    logger.debug("Get service manager dependency called")
    manager = ServiceManager()
    return manager


@lru_cache()
def get_bag_manager() -> BagRecordingManager:
    """Get a singleton instance of the BagManager.

    Initializes the manager using the configured storage path from device settings.

    Returns:
        BagManager: Manages ROS bag recordings.
    """
    logger.debug("Get bag manager dependency called")
    manager = BagRecordingManager(get_device_settings().bags_storage_path)
    return manager


@lru_cache()
def get_topic_monitor_runner() -> Optional["TopicMonitorRunner"]:
    """Get a singleton instance of the TopicMonitorRunner.

    Initializes the monitor with topics configured in device settings.

    Returns:
        TopicMonitorRunner: Continuously monitors ROS topic activity.
    """
    logger.debug("Get bag manager dependency called")

    try:
        runner = create_topic_monitor_runner(
            get_device_settings().ros.topics_to_monitor
        )
        return runner
    except RuntimeError:
        return None
