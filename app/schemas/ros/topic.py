from pydantic import BaseModel, Field
from typing import Optional


class TopicBase(BaseModel):
    """Base model representing a ROS 2 topic with essential information."""

    name: str = Field(description="Name of the topic (e.g., `/cmd_vel`)")
    msg_type: str = Field(
        description="The message type of the topic (e.g., `geometry_msgs/msg/Twist`)."
    )


class TopicStatus(TopicBase):
    """Extended topic model including status and frequency information."""

    hz: Optional[float] = Field(
        default=None,
        description="The observed frequency (in Hz) at which messages are published.",
    )
    status: Optional[str] = Field(
        default=None, description="The current status of the topic. (Online, Offline)"
    )
    subscribers: Optional[int] = Field(
        default=None, description="The current number of subscribers"
    )
