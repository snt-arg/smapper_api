from collections import defaultdict
import threading
from enum import Enum
from typing import Any, Dict, List, Set, Tuple
from pydantic import BaseModel, Field
import importlib
import time
from app.logging import logger

import rclpy
from rclpy.node import Node, Subscription


class TopicStatus(Enum):
    """Enum representing the status of a ros topic.

    Members:
        ONLINE: Topic is currently receiving messages
        OFFLINE: Topic is not receiving any messages
    """

    ONLINE = "ONLINE"
    OFFLINE = "OFFLINE"


class TopicState(BaseModel):
    """Represents the state of a topic, including its status, frequency,
    message type, and message metrics.

    Attributes:
        status: The current status of the topic (e.g., online or offline).
        hz: Frequency in hertz, indicating how often messages are published.
        msg_type: The type of message associated with the topic.
        last_rcv_ts: Timestamp (in nanoseconds) for the last received message.
        message_count: Total number of messages that have been processed.
    """

    status: TopicStatus = Field(default=TopicStatus.OFFLINE)
    hz: float = Field(default=0.0)
    msg_type: str
    last_rcv_ts: float
    message_count: int


class TopicMonitor(Node):
    """A ROS2 node which monitors topics and tracks their state.

    This node subscribes to designated topics and monitors metrics such as message frequency,
    last received timestamp, and total message count using a TopicState model.
    """

    _topic_states: Dict[str, TopicState]
    _topic_subs: Dict[str, Subscription]
    _topic_timeout: float  # seconds
    _topics_to_monitor: Set[str]

    def __init__(
        self,
        node_name: str,
        topic_list: List[str],
        monitor_rate: float = 1,
        discover_rate: float = 2,
        topic_timeout: float = 3,
    ) -> None:
        """Initialize the TopicMonitor node.

        Args:
            node_name: The name of the ROS2 node.
            topic_list: A list of topics to monitor.
            monitor_rate: The rate (in seconds) at which to check topic statuses.
            discover_rate: The rate (in seconds) at which to discover new topics.
            topic_timeout: The timeout period (in seconds) after which a topic is considered offline.
        """
        super().__init__(node_name)
        self._topic_states = defaultdict()
        self._topic_subs = defaultdict()
        self._topic_timeout = topic_timeout
        self._topics_to_monitor = set(topic_list)

        self.create_timer(monitor_rate, self._monitor_topics)
        self.create_timer(discover_rate, self._discover_new_topics)

        logger.info("Topic Monitor initialized")

    def get_topic_states(self) -> Dict[str, TopicState]:
        """Return the current states of all monitored topics.

        Returns:
            A dictionary mapping topic names to their respective TopicState instances.
        """
        return self._topic_states

    def add_topic_to_monitor(self, topic: str) -> None:
        """Add a single topic to the set of topics to monitor.

        Args:
            topic: The topic name to add.
        """
        self._topics_to_monitor.add(topic)

    def add_topics_to_monitor(self, topics: List[str]) -> None:
        """Add multiple topics to the set of topics to monitor.

        Args:
            topics: A list of topic names to add.
        """
        self._topics_to_monitor.update(topics)

    def remove_topic_from_monitor(self, topic: str) -> None:
        """Safely remove a topic from monitoring.

        This method removes the topic from the monitoring set, unsubscribes from it,
        and deletes its state information.

        Args:
            topic: The topic name to remove.
        """
        try:
            if topic in self._topics_to_monitor:
                self._topics_to_monitor.remove(topic)

            if topic in self._topic_subs:
                sub = self._topic_subs[topic]

                try:
                    if sub in self._subscriptions:
                        self._subscriptions.remove(sub)
                except Exception as e:
                    logger.warning(f"Error removing subscription from node: {e}")

                del self._topic_subs[topic]

            if topic in self._topic_states:
                del self._topic_states[topic]

            logger.info(f"Removed topic {topic} from monitoring")

        except Exception as e:
            logger.error(f"Failed to remove topic {topic}: {e}")

    def remove_topics_from_monitor(self, topics: List[str]) -> None:
        """Safely remove multiple topics from monitoring.

        Args:
            topics: A list of topic names to remove.
        """
        for topic in topics:
            self.remove_topic_from_monitor(topic)

    def _discover_new_topics(self) -> None:
        """Discover new topics and subscribe to those not currently tracked.

        This method checks the available topics and, if a topic is in the monitoring list
        but not already being tracked, adds it to the subscription process.
        """
        available_topics = self.get_topic_names_and_types()
        untracked = []

        for topic, msg_type in available_topics:
            if (
                self._topic_states.get(topic) is None
                and topic in self._topics_to_monitor
            ):
                untracked.append((topic, msg_type))

        self._subscribe_to_topics(untracked)

    def _subscribe_to_topics(
        self, topics_and_types: List[Tuple[str, List[str]]]
    ) -> None:
        """Subscribe to a list of topics along with their associated message types.

        Args:
            topics_and_types: A list of tuples, each containing a topic name and a list of message types.
        """
        for topic_name, msg_types in topics_and_types:
            assert len(msg_types) > 0

            # NOTE: Only considering the first available topic type
            msg_type_str = msg_types[0]

            try:
                msg_type = self._get_message_class(msg_type_str)
            except ImportError as e:
                logger.error(f"Failed to import message type for {topic_name}: {e}")
                continue

            sub = self.create_subscription(
                msg_type,
                topic_name,
                lambda msg, topic_name=topic_name: self._topic_callback(
                    msg, topic_name
                ),
                1,
            )

            self._topic_subs[topic_name] = sub

            self._topic_states[topic_name] = TopicState(
                status=TopicStatus.OFFLINE,
                msg_type=msg_type_str,
                hz=0,
                last_rcv_ts=0,
                message_count=0,
            )

            logger.info(
                f"[ROS_MONITOR] Subscribed to {topic_name} with message type {msg_type}"
            )

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

    def _topic_callback(self, _: Any, topic_name: str) -> None:
        """Callback function for incoming messages on a topic.

        Updates the TopicState for the given topic with the current timestamp,
        calculates message frequency, and increments the message count.

        Args:
            _: The incoming message (unused).
            topic_name: The name of the topic from which the message was received.
        """
        current_time = time.time()
        time_diff = current_time - self._topic_states[topic_name].last_rcv_ts
        state = self._topic_states[topic_name]

        state.status = TopicStatus.ONLINE
        state.hz = 1.0 / time_diff if time_diff > 0 else 0
        state.last_rcv_ts = current_time
        state.message_count += 1
        # TODO: Add the count of subscribers
        # state.subscribers = self.count_subscribers(topic_name) - 1

    def _monitor_topics(self) -> None:
        """Monitor the topics and update their status.

        This method periodically checks if any topic has exceeded the timeout period
        since its last received message, marking such topics as offline.
        """
        for topic_name, state in self._topic_states.items():
            current_time = time.time()
            if current_time - state.last_rcv_ts > self._topic_timeout:
                state.status = TopicStatus.OFFLINE
                state.hz = 0

            logger.debug(
                f"Topic: {topic_name}, Status: {state.status}, Frequency: {state.hz:.2f} Hz"
            )


class TopicMonitorRunner:
    """Runner for the TopicMonitor node.

    This class manages the lifecycle of a TopicMonitor node in a separate thread,
    providing methods to start, stop, and interact with the monitored topic states.
    """

    def __init__(self, topics_to_monitor: List[str]):
        """Initialize the TopicMonitorRunner.

        Args:
            topics_to_monitor: A list of topics that the TopicMonitor should initially monitor.
        """
        self._thread = threading.Thread(target=self.__ros_spin)
        self._running = False
        self.topic_states: Dict[str, TopicState] = {}

        rclpy.init()
        self._node = TopicMonitor("api_topic_monitor", topics_to_monitor)

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

    def get_all_topic_states(self) -> Dict[str, TopicState]:
        """Retrieve the states of all monitored topics.

        Returns:
            A dictionary mapping topic names to TopicState instances.
        """
        return self._node.get_topic_states()

    def get_topic_state(self, topic_name: str) -> TopicState | None:
        """Retrieve the state of a specific topic.

        Args:
            topic_name: The name of the topic to query.

        Returns:
            The TopicState for the given topic, or None if not found.
        """
        return self._node.get_topic_states().get(topic_name)

    def add_topic_to_monitor(self, topic: str) -> None:
        """Add a single topic to be monitored.

        Args:
            topic: The name of the topic to add.
        """
        self._node.add_topic_to_monitor(topic)

    def add_topics_to_monitor(self, topics: List[str]) -> None:
        """Add multiple topics to be monitored.

        Args:
            topics: A list of topic names to add.
        """
        self._node.add_topics_to_monitor(topics)

    def remove_topic_from_monitor(self, topic: str) -> None:
        """Remove a single topic from monitoring.

        Args:
            topic: The name of the topic to remove.
        """
        self._node.remove_topic_from_monitor(topic)

    def remove_topics_from_monitor(self, topics: List[str]) -> None:
        """Remove multiple topics from monitoring.

        Args:
            topics: A list of topic names to remove.
        """
        self._node.remove_topics_from_monitor(topics)

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
