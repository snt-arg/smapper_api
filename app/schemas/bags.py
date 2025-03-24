import datetime
from typing import List, Optional
from pydantic import BaseModel
from .ros import TopicSchema


class MinimalRosbagMetadata(BaseModel):
    duration: float
    unix_timestamp: float
    topics: List[TopicSchema]
    message_count: int
    db_path: str


class BagSchema(BaseModel):
    id: str
    name: str
    bag_size: str
    rosbag_metadata: MinimalRosbagMetadata
    detail: Optional[str] = ""
    tags: Optional[List[str]] = []
