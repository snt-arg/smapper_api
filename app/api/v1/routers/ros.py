from typing import Annotated, List, TYPE_CHECKING
from fastapi import APIRouter, Depends
from app.di import get_topic_monitor_runner
from app.schemas.ros.topic import TopicStatus


if TYPE_CHECKING:
    from app.core.ros.topic_monitor import TopicMonitorRunner


router = APIRouter(prefix="/api/v1")


@router.get(
    "/ros/topics",
    description="Get a list of current ROS topics being monitored, along with their state (status, frequency, message count, etc).",
)
def get_ros_topics(
    runner: Annotated["TopicMonitorRunner", Depends(get_topic_monitor_runner)],
) -> List[TopicStatus]:
    states = []
    if runner:
        for name, state in runner.get_all_topic_states().items():
            states.append(
                TopicStatus(
                    name=name,
                    msg_type=state.msg_type,
                    hz=state.hz,
                    status=state.status.value,
                )
            )
    return states


@router.get(
    "/ros/topics/{topic_name}",
    description="Get the current monitoring state for a specific ROS topic by its name.",
)
def get_topic_by_name(
    topic_name: str,
    runner: Annotated["TopicMonitorRunner", Depends(get_topic_monitor_runner)],
) -> TopicStatus | None:
    if runner:
        state = runner.get_topic_state(topic_name)
        if state is None:
            return None
        return TopicStatus(
            name=topic_name,
            msg_type=state.msg_type,
            hz=state.hz,
            message_count=state.message_count,
            status=state.status.value,
        )


@router.post(
    "/ros/topics/",
    description="Add a new topic to be monitored. If the topic becomes available, it will be automatically subscribed to.",
)
def add_topic_to_monitor(
    topic_name: str,
    runner: Annotated["TopicMonitorRunner", Depends(get_topic_monitor_runner)],
) -> None:
    if runner:
        runner.add_topic_to_monitor(topic_name)


@router.delete(
    "/ros/topics/",
    description="Remove a topic from the monitoring list. If subscribed, it will be unsubscribed and cleaned up.",
)
def remove_topic_to_monitor(
    topic_name: str,
    runner: Annotated["TopicMonitorRunner", Depends(get_topic_monitor_runner)],
) -> None:
    if runner:
        runner.remove_topic_from_monitor(topic_name)


# @router.post(
#     "/ros/topics/batch",
#     description="Add multiple topics to the monitor list in a single request.",
# )
# def add_topics_to_monitor(
#     topic_names: List[str],
#     runner: Annotated[TopicMonitorRunner, Depends(get_topic_monitor_runner)],
# ) -> None:
#     runner.add_topics_to_monitor(topic_names)


# @router.delete(
#     "/ros/topics/batch",
#     description="Remove multiple topics from the monitor list in a single request.",
# )
# def remove_topics_to_monitor(
#     topic_names: List[str],
#     runner: Annotated[TopicMonitorRunner, Depends(get_topic_monitor_runner)],
# ) -> None:
#     runner.remove_topics_from_monitor(topic_names)
