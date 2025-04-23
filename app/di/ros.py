from functools import lru_cache
from typing import TYPE_CHECKING, Optional

from app.core.managers import RecordingManager
from app.core.ros.ros_factory import create_topic_monitor_runner
from .settings import get_managers_settings
from app.logging import logger


if TYPE_CHECKING:
    from app.core.ros.topic_monitor import TopicMonitorRunner


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
            get_managers_settings().ros.topic_monitor.topics_blacklist
        )
        return runner
    except RuntimeError:
        return None


@lru_cache()
def get_recording_manager() -> RecordingManager:
    """Get a singleton instance of the RecordingManager.

    Initializes the manager using the configured storage path from device settings.

    Returns:
        BagManager: Manages ROS bag recordings.
    """
    logger.debug("Get recording manager dependency called")
    manager = RecordingManager(get_managers_settings().ros.bag_recorder.storage_path)
    return manager
