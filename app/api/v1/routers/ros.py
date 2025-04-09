from typing import Annotated, List, TYPE_CHECKING
from fastapi import APIRouter, Depends
from app.di import get_topic_monitor_runner
from app.schemas.ros.topic import TopicStatus


if TYPE_CHECKING:
    from app.core.ros.topic_monitor import TopicMonitorRunner


router = APIRouter(prefix="/api/v1/ros")


@router.get(
    "/topics",
    description="Get a list of current ROS topics being monitored, along with their state (status, frequency, message count, etc).",
    tags=["ros", "topics"],
)
def get_topics(
    runner: Annotated["TopicMonitorRunner", Depends(get_topic_monitor_runner)],
) -> List[TopicStatus]:
    if runner:
        return runner.get_topics()
    return []


@router.get(
    "/topics/{topic_name}",
    description="Get the current monitoring state for a specific ROS topic by its name.",
    tags=["ros", "topics"],
)
def get_topic(
    topic_name: str,
    runner: Annotated["TopicMonitorRunner", Depends(get_topic_monitor_runner)],
) -> TopicStatus | None:
    if runner:
        return runner.get_topic(topic_name)
    return []


@router.post(
    "/topics/",
    description="Add a new topic to be monitored. If the topic becomes available, it will be automatically subscribed to.",
    tags=["ros", "topics"],
)
def add_topic(
    topic_name: str,
    runner: Annotated["TopicMonitorRunner", Depends(get_topic_monitor_runner)],
) -> None:
    if runner:
        runner.add_topic_to_monitor(topic_name)


@router.delete(
    "/topics/",
    description="Remove a topic from the monitoring list. If subscribed, it will be unsubscribed and cleaned up.",
    tags=["ros", "topics"],
)
def remove_topic(
    topic_name: str,
    runner: Annotated["TopicMonitorRunner", Depends(get_topic_monitor_runner)],
) -> None:
    if runner:
        runner.remove_topic_from_monitor(topic_name)
