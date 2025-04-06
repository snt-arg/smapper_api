from pydantic import BaseModel
from typing import List, Optional

from app.schemas.ros import Topic


class RosbagTopic(Topic):
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
    topics: List[Topic] = []
    rosbag_path: str


class RosbagMetadata(RosbagMetadataBase):
    id: int
    topics: List[RosbagTopic] = []

    class Config:
        from_attributes = True
