from typing import Annotated, List, TYPE_CHECKING, Optional, Tuple
from fastapi import APIRouter, Depends
from app.core.exceptions import RosNotAvailable
from app.di import get_topic_monitor_runner
from app.schemas.ros.topic import TopicStatus


if TYPE_CHECKING:
    from app.core.ros.topic_monitor import TopicMonitorRunner


router = APIRouter(prefix="/api/v1/ros")


@router.get(
    "/topics/",
    description="Get a list of current ROS topics being monitored, along with their state (status, frequency, message count, etc).",
    tags=["ros", "topics"],
)
def get_topics(
    runner: Annotated[
        Optional["TopicMonitorRunner"], Depends(get_topic_monitor_runner)
    ],
    all: bool = False,
) -> List[TopicStatus]:
    if runner:
        if all:
            return runner.get_all_topics()
        return runner.get_tracked_topic_states()
    raise RosNotAvailable()
