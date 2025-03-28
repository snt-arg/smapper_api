from collections import defaultdict
import threading
from enum import Enum
from typing import Any, Dict, List, Set, Tuple
from pydantic import BaseModel, Field
import rclpy
from rclpy.node import Node, Subscription
import importlib
import time
from app.logger import logger


class TopicStatus(Enum):
    ACTIVE = "ACTIVE"
    INACTIVE = "INACTIVE"


class TopicState(BaseModel):
    status: TopicStatus = Field(default=TopicStatus.INACTIVE)
    hz: float = Field(default=0.0)
    msg_type: str
    last_rcv_ts: float
    message_count: int


class TopicMonitor(Node):
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
        super().__init__(node_name)
        self._topic_states = defaultdict()
        self._topic_subs = defaultdict()
        self._topic_timeout = topic_timeout
        self._topics_to_monitor = set(topic_list)

        self.create_timer(monitor_rate, self._monitor_topics)
        self.create_timer(discover_rate, self._discover_new_topics)

        logger.info("Topic Monitor initialized")

    def get_topic_states(self) -> Dict[str, TopicState]:
        return self._topic_states

    def add_topic_to_monitor(self, topic: str) -> None:
        self._topics_to_monitor.add(topic)

    def add_topics_to_monitor(self, topics: List[str]) -> None:
        self._topics_to_monitor.update(topics)

    def remove_topic_from_monitor(self, topic: str) -> None:
        """
        Safely remove a topic from monitoring.
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
        for topic in topics:
            self.remove_topic_from_monitor(topic)

    def _discover_new_topics(self) -> None:
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
        for topic_name, msg_types in topics_and_types:
            assert len(msg_types) > 0

            # Only considering the first available topic type
            msg_type_str = msg_types[0]

            # Dynamically load msg type
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
                status=TopicStatus.INACTIVE,
                msg_type=msg_type_str,
                hz=0,
                last_rcv_ts=0,
                message_count=0,
            )

            logger.info(
                f"[ROS_MONITOR] Subscribed to {topic_name} with message type {msg_type}"
            )

    def _get_message_class(self, msg_type_str: str) -> Any:
        """Dynamically import a message class based on the type string"""
        msg_type_split = msg_type_str.split("/")
        package_name, msg_name = ".".join(msg_type_split[:-1]), msg_type_split[-1]

        try:
            module = importlib.import_module(package_name)
            msg_class = getattr(module, msg_name)
            return msg_class
        except (ImportError, AttributeError) as e:
            raise ImportError(f"Could not load message class for {msg_type_str}: {e}")

    def _topic_callback(self, _: Any, topic_name: str) -> None:
        current_time = time.time()
        time_diff = current_time - self._topic_states[topic_name].last_rcv_ts
        state = self._topic_states[topic_name]

        state.status = TopicStatus.ACTIVE
        state.hz = 1.0 / time_diff if time_diff > 0 else 0
        state.last_rcv_ts = current_time
        state.message_count += 1

        logger.debug(f"Received msg from {topic_name}")

    def _monitor_topics(self) -> None:
        """Monitor the topics and log the status"""
        for topic_name, state in self._topic_states.items():
            current_time = time.time()
            if current_time - state.last_rcv_ts > self._topic_timeout:
                state.status = TopicStatus.INACTIVE
                state.hz = 0

            # Log topic status and frequency (hz)
            logger.debug(
                f"Topic: {topic_name}, Status: {state.status}, Frequency: {state.hz:.2f} Hz"
            )


class TopicMonitorRunner:
    def __init__(self, topics_to_monitor: List[str]):
        self._thread = threading.Thread(target=self.__ros_spin)
        self._running = False
        self.topic_states: Dict[str, TopicState] = {}

        rclpy.init()
        self._node = TopicMonitor("api_topic_monitor", topics_to_monitor)

    def start(self):
        self._running = True
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join()
        rclpy.try_shutdown()

    def get_all_topic_states(self) -> Dict[str, TopicState]:
        if self._node:
            return self._node.get_topic_states()
        return {}

    def get_topic_state(self, topic_name: str) -> TopicState | None:
        return self._node.get_topic_states().get(topic_name)

    def add_topic_to_monitor(self, topic: str) -> None:
        self._node.add_topic_to_monitor(topic)

    def add_topics_to_monitor(self, topics: List[str]) -> None:
        self._node.add_topics_to_monitor(topics)

    def remove_topic_from_monitor(self, topic: str) -> None:
        self._node.remove_topic_from_monitor(topic)

    def remove_topics_from_monitor(self, topics: List[str]) -> None:
        self._node.remove_topics_from_monitor(topics)

    def __ros_spin(self):
        while rclpy.ok() and self._running:
            try:
                rclpy.spin_once(self._node)
            except:
                logger.error("Failed to spin Topic Monitor Ros node")
                break
        if self._node:
            self._node.destroy_node()
