from functools import lru_cache

from app.config.settings import APISettings, AppSettings, DeviceSettings
from app.core.ros.topic_monitor import TopicMonitorRunner
from app.core.service_manager import ServiceManager
from app.core.bag_manager import BagManager
from app.logger import logger


@lru_cache()
def get_app_settings() -> AppSettings:
    logger.debug("Get App settings dependency called")
    return AppSettings()


@lru_cache()
def get_device_settings() -> DeviceSettings:
    logger.debug("Get device settings dependency called")
    return get_app_settings().device


@lru_cache()
def get_api_settings() -> APISettings:
    logger.debug("Get API settings dependency called")
    return get_app_settings().api


@lru_cache()
def get_service_manager() -> ServiceManager:
    logger.debug("Get service manager dependency called")
    manager = ServiceManager()
    return manager


@lru_cache()
def get_bag_manager() -> BagManager:
    logger.debug("Get bag manager dependency called")
    manager = BagManager(get_device_settings().bags_storage_path)
    return manager


@lru_cache()
def get_topic_monitor_runner() -> TopicMonitorRunner:
    logger.debug("Get bag manager dependency called")
    runner = TopicMonitorRunner(get_device_settings().ros.topics_to_monitor)
    return runner
