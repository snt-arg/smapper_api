from typing import List, TYPE_CHECKING

if TYPE_CHECKING:
    from .topic_monitor import TopicMonitorRunner


def create_topic_monitor_runner(topics_to_monitor: List[str]) -> "TopicMonitorRunner":
    """
    Create and return an instance of TopicMonitorRunner if ROS2 is available.

    This function attempts to import the ROS2-dependent `TopicMonitorRunner` class.
    If ROS2 (via the `rclpy` module) is not installed or cannot be imported, it raises
    a RuntimeError. This allows the rest of the application to remain functional even
    if ROS2 is not present.

    Args:
        topics_to_monitor (List[str]): A list of ROS topic names to monitor.

    Returns:
        TopicMonitorRunner: An instance of the TopicMonitorRunner class.

    Raises:
        RuntimeError: If ROS2 or its dependencies are not available.
    """
    try:
        from .topic_monitor import TopicMonitorRunner

        return TopicMonitorRunner(topics_to_monitor)
    except ImportError:
        raise RuntimeError("ROS2 is not installed or available. Unable to import rclpy")
