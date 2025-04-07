from pydantic import BaseModel


class TopicBase(BaseModel):
    """Represention of a ROS topic.

    Attributes:

        name: The name of the ROS topic (e.g., "/cmd_vel").
        msg_type: The message type of the topic (e.g., "geometry_msgs/msg/Twist").
    """

    name: str
    msg_type: str


class TopicStatus(TopicBase):
    """Schema representing metadata for a ROS topic.

    Attributes:

        name: The name of the ROS topic (e.g., "/cmd_vel").
        msg_type: The message type of the topic (e.g., "geometry_msgs/msg/Twist").
        hz: The observed frequency (in Hz) at which messages are published.
        status: The current status of the topic (e.g., "ACTIVE", "INACTIVE").
    """

    hz: float
    status: str
