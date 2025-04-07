from pydantic import BaseModel
from typing import List, Optional

from app.schemas.ros.topic import TopicBase


class RosbagTopic(TopicBase):
    id: int

    class Config:
        from_attributes = True


class RosbagMetadataBase(BaseModel):
    name: str
    size: Optional[int] = None
    duration: Optional[float] = None
    start_time: Optional[float] = None
    end_time: Optional[float] = None
    detail: Optional[str] = None
    tags: Optional[str] = None


class RosbagMetadataCreate(RosbagMetadataBase):
    topics: List[TopicBase] = []
    rosbag_path: str


class RosbagMetadataUpdate(BaseModel):
    name: Optional[str] = None
    detail: Optional[str] = None
    tags: Optional[str] = None


class RosbagMetadata(RosbagMetadataBase):
    id: int
    topics: List[RosbagTopic] = []

    class Config:
        from_attributes = True


class RosbagFileMetadata(BaseModel):
    """Minimal representation of metadata.yaml of ROS2 bag.

    Attributes:
        duration: Duration of the recording in nanoseconds.
        unix_timestamp: Start time of the recording in epoch nanoseconds.
        topics: List of topics recorded in the bag.
        message_count: Total number of messages recorded.
        db_path: Relative path to the underlying database file.
    """

    duration: float
    start_time: float
    topics: List[TopicBase]
    message_count: int
    db_path: str
