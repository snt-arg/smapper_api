from pydantic import BaseModel, Field


class TopicBase(BaseModel):
    """Base model representing a ROS 2 topic with essential information."""

    name: str = Field(description="Name of the topic (e.g., `/cmd_vel`)")
    msg_type: str = Field(
        description="The message type of the topic (e.g., `geometry_msgs/msg/Twist`)."
    )


class TopicStatus(TopicBase):
    """Extended topic model including status and frequency information."""

    hz: float = Field(
        description="The observed frequency (in Hz) at which messages are published."
    )
    status: str = Field(
        description="The current status of the topic. (Online, Offline)"
    )
    subscribers: int = Field(description="The current number of subscribers")
