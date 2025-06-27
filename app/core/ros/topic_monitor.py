import re
from collections import defaultdict
import threading
from enum import Enum
from typing import Any, Dict, List, Set, Tuple
from pydantic import BaseModel, Field
import importlib
import time

from app.logging import logger
from app.schemas.ros.topic import TopicStatus as TopicStatusSchema

import rclpy
from rclpy.node import Node, Subscription


class TopicStatus(Enum):
    """Enum representing the status of a ros topic.

    Members:
        ONLINE: Topic is currently receiving messages
        OFFLINE: Topic is not receiving any messages
    """

    ONLINE = "Online"
    OFFLINE = "Offline"


class TopicState(BaseModel):
    """Represents the state of a topic, including its status, frequency,
    message type, and message metrics.

    Attributes:
        status: The current status of the topic (e.g., online or offline).
        hz: Frequency in hertz, indicating how often messages are published.
        msg_type: The type of message associated with the topic.
        last_rcv_ts: Timestamp (in nanoseconds) for the last received message.
        subscribers: Current number of subscribers of the topic
        message_count: Total number of messages between polling.
    """

    status: TopicStatus = Field(default=TopicStatus.OFFLINE)
    hz: float = Field(default=0.0)
    msg_type: str
    prev_monitor_time: float
    subscribers: int
    message_count: int


class TopicMonitor(Node):
    """A ROS2 node which monitors topics and tracks their state."""

    _discovery_polling_interval: float
    _monitoring_polling_interval: float
    _untrack_polling_interval: float
    _blacklist: Set[str]
    _subs: Dict[str, Subscription]
    _tracked_topics: Dict[str, TopicState]
    _to_untrack_topics: Set[str]
    _available_topics: Set[Tuple[str, str]]
    _accepted_timeout: float
    _topics_to_monitor: Set[str]

    def __init__(
        self,
        topics_to_monitor: List[str],
        blacklist: List[str],
        discovery_polling_interval: float = 5,
        monitoring_polling_interval: float = 2,
        accepted_timeout: float = 3,
    ):
        self._topics_to_monitor = set(topics_to_monitor)
        self._blacklist = set(blacklist)
        self._tracked_topics = defaultdict()
        self._available_topics = set()

        self._discovery_polling_interval = discovery_polling_interval
        self._monitoring_polling_interval = monitoring_polling_interval
        self._accepted_timeout = accepted_timeout

        super().__init__("topic_monitor")

        self.create_timer(self._discovery_polling_interval, self._discovery_cb)
        self.create_timer(self._monitoring_polling_interval, self._monitoring_cb)

    def get_tracked_topic_states(self) -> List[TopicStatusSchema]:
        """Gets all topic states that are currently being tracked.

        Returns:
            List[TopicState]: list of all tracked topic states
        """
        return [
            TopicStatusSchema(
                name=name,
                msg_type=state.msg_type,
                hz=state.hz,
                status=state.status.value,
                subscribers=state.subscribers,
            )
            for name, state in self._tracked_topics.items()
        ]

    def get_all_topics(self) -> List[TopicStatusSchema]:
        """Gets all topics that are currently available (not blacklisted), both tracked and untracked.

        If a topic is being tracked, it's additional infomation becomes available,
        like hz.

        Returns:
            List[TopicStatusSchema]: list of tuples where topic=(topic name, type)
        """
        topics = []
        for name, msg_type in self._available_topics:
            if name in self._blacklist:
                continue
            if name in self._tracked_topics:
                topic = self._tracked_topics[name]
                topics.append(
                    TopicStatusSchema(
                        name=name,
                        msg_type=topic.msg_type,
                        hz=topic.hz,
                        status=topic.status.value,
                        subscribers=topic.subscribers,
                    )
                )
            else:
                topics.append(TopicStatusSchema(name=name, msg_type=msg_type))
        return topics

    def _discovery_cb(self):
        """Discovers new topics and subscribe to those not currently tracked.

        This method checks the available topics and, if a topic is in the monitoring list
        but not already being tracked, adds it to the subscription process.
        """
        self._available_topics = {
            (name, msg_type[0]) for name, msg_type in self.get_topic_names_and_types()
        }

        for topic_name, msg_type in self._available_topics:
            if (
                self._is_topic_blacklisted(topic_name)
                or topic_name in self._tracked_topics
                or topic_name not in self._topics_to_monitor
            ):
                continue
            if not self._tracked_topics.get(topic_name):
                self._subscribe_topic(topic_name, msg_type)

    def _monitoring_cb(self):
        """Monitor the topics and update their status.

        This method periodically checks if any topic has exceeded the timeout period
        since its last received message, marking such topics as offline.
        """
        for topic_name, state in self._tracked_topics.items():
            current_time = time.time()
            time_diff = current_time - state.prev_monitor_time

            if state.message_count == 0 and state.status is not TopicStatus.OFFLINE:
                state.status = TopicStatus.OFFLINE
                state.hz = 0
                continue

            state.hz = state.message_count / time_diff

            state.message_count = 0
            state.prev_monitor_time = current_time

            logger.debug(
                f"Topic: {topic_name}, Status: {state.status}, Frequency: {state.hz:.2f} Hz"
            )

    def _topic_callback(self, _: Any, topic_name: str) -> None:
        """Callback function for incoming messages on a topic.

        Updates the TopicState for the given topic.

        Args:
            _: The incoming message (unused).
            topic_name: The name of the topic from which the message was received.
        """
        state = self._tracked_topics[topic_name]

        state.status = TopicStatus.ONLINE
        state.message_count += 1
        state.subscribers = self.count_subscribers(topic_name) - 1

    def _subscribe_topic(self, topic_name: str, msg_type_str: str):
        try:
            msg_type = self._get_message_class(msg_type_str)
        except ImportError as e:
            logger.error(f"Failed to import message type for {topic_name}: {e}")
            return

        self.create_subscription(
            msg_type,
            topic_name,
            lambda msg, topic_name=topic_name: self._topic_callback(msg, topic_name),
            1,
        )

        self._tracked_topics[topic_name] = TopicState(
            status=TopicStatus.OFFLINE,
            msg_type=msg_type_str,
            hz=0,
            prev_monitor_time=0,
            subscribers=0,
            message_count=0,
        )

        logger.info(f"Subscribed to {topic_name}")

    def _get_message_class(self, msg_type_str: str) -> Any:
        """Dynamically import a message class based on the type string.

        Args:
            msg_type_str: A string in the format 'package/msg/MessageType' representing the message type.

        Returns:
            The message class corresponding to the provided type string.

        Raises:
            ImportError: If the message class cannot be imported.
        """
        msg_type_split = msg_type_str.split("/")
        package_name, msg_name = ".".join(msg_type_split[:-1]), msg_type_split[-1]

        try:
            module = importlib.import_module(package_name)
            msg_class = getattr(module, msg_name)
            return msg_class
        except (ImportError, AttributeError) as e:
            raise ImportError(f"Could not load message class for {msg_type_str}: {e}")

    def _is_topic_blacklisted(self, name: str) -> bool:
        for blacklisted in self._blacklist:
            if re.match(name, blacklisted):
                return True
        return False


class TopicMonitorRunner:
    """Runner for the TopicMonitor node.

    This class manages the lifecycle of a TopicMonitor node in a separate thread,
    providing methods to start, stop, and interact with the monitored topic states.
    """

    def __init__(
        self,
        topics_to_monitor: List[str],
        topics_blacklist: List[str],
        monitor_rate: float = 1,
        discover_rate: float = 2,
        idle_timeout: float = 3,
    ):
        """Initialize the TopicMonitorRunner.

        Args:
            topics_to_monitor: A list of topics that the TopicMonitor should initially monitor.
        """
        self._thread = threading.Thread(target=self.__ros_spin)
        self._running = False
        self.topic_states: Dict[str, TopicState] = {}

        rclpy.init()
        self._node = TopicMonitor(
            topics_to_monitor, topics_blacklist, monitor_rate, discover_rate, idle_timeout
        )

    def start(self):
        """Start the TopicMonitorRunner thread."""
        self._running = True
        self._thread.start()

    def stop(self):
        """Stop the TopicMonitorRunner thread and shutdown the ROS node."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join()
        rclpy.try_shutdown()

    def get_tracked_topic_states(self) -> List[TopicStatusSchema]:
        """Retrieve the states of all tracked topics.

        Returns:
            List[TopicStatusSchema]:
        """
        return self._node.get_tracked_topic_states()

    def get_all_topics(self) -> List[TopicStatusSchema]:
        """Retrieve the states of all available topics, even untracked.

        Returns:
            List[TopicStatusSchema]:
        """
        return self._node.get_all_topics()

    def __ros_spin(self):
        """Spin the ROS node in a separate thread until stopped.

        Continuously processes ROS events and ensures that the node is responsive.
        """
        while rclpy.ok() and self._running:
            try:
                rclpy.spin_once(self._node)
            except:
                logger.error("Failed to spin Topic Monitor Ros node")
                break
        if self._node:
            try:
                self._node.destroy_node()
            except Exception as e:
                logger.error(f"Error destroying node: {e}")
