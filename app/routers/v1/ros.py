from typing import Annotated, List
from fastapi import APIRouter, Depends
from app.core.ros.topic_monitor import TopicMonitorRunner
from app.dependencies import get_topic_monitor_runner
from app.schemas.ros import TopicSchema


router = APIRouter(prefix="/api/v1")


@router.get("/ros/topics", description="Get a list topics states")
def get_ros_topics(
    runner: Annotated[TopicMonitorRunner, Depends(get_topic_monitor_runner)],
) -> List[TopicSchema]:
    states = []
    for name, state in runner.get_all_topic_states().items():
        states.append(
            TopicSchema(
                name=name,
                msg_type=state.msg_type,
                hz=state.hz,
                message_count=state.message_count,
                status=state.status.value,
            )
        )
    return states


@router.get("/ros/topics/{topic_name}", description="Get the topic state by the name")
def get_topic_by_name(
    topic_name: str,
    runner: Annotated[TopicMonitorRunner, Depends(get_topic_monitor_runner)],
) -> TopicSchema | None:
    state = runner.get_topic_state(topic_name)
    if state is None:
        return None
    return TopicSchema(
        name=topic_name,
        msg_type=state.msg_type,
        hz=state.hz,
        message_count=state.message_count,
        status=state.status.value,
    )


@router.post("/ros/topics/", description="Get the topic state by the name")
def add_topic_to_monitor(
    topic_name: str,
    runner: Annotated[TopicMonitorRunner, Depends(get_topic_monitor_runner)],
) -> None:
    runner.add_topic_to_monitor(topic_name)


@router.delete("/ros/topics/", description="Get the topic state by the name")
def remove_topic_to_monitor(
    topic_name: str,
    runner: Annotated[TopicMonitorRunner, Depends(get_topic_monitor_runner)],
) -> None:
    runner.remove_topic_from_monitor(topic_name)


# @router.post("/ros/topics/", description="Get the topic state by the name")
# def add_topics_to_monitor(
#     topic_names: List[str],
#     runner: Annotated[TopicMonitorRunner, Depends(get_topic_monitor_runner)],
# ) -> None:
#     runner.add_topics_to_monitor(topic_names)


# @router.delete("/ros/topics/", description="Get the topic state by the name")
# def remove_topics_to_monitor(
#     topic_names: List[str],
#     runner: Annotated[TopicMonitorRunner, Depends(get_topic_monitor_runner)],
# ) -> None:
#     runner.remove_topics_from_monitor(topic_names)
