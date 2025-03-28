from typing import Optional
from pydantic import BaseModel


class TopicSchema(BaseModel):
    """Schema representing metadata for a ROS topic.

    Attributes:
        name: The name of the ROS topic (e.g., "/cmd_vel").
        msg_type: The message type of the topic (e.g., "geometry_msgs/msg/Twist").
        hz: The observed frequency (in Hz) at which messages are published.
        message_count: Optional total number of messages received on the topic.
        status: The current status of the topic (e.g., "ACTIVE", "INACTIVE").
    """

    name: str
    msg_type: str
    hz: float
    message_count: Optional[int]
    status: str
