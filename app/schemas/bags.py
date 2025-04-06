from typing import List, Optional
from pydantic import BaseModel
from .ros import Topic


class MinimalRosbagMetadata(BaseModel):
    """Minimal representation of ROS2 bag metadata.

    Attributes:
        duration: Duration of the recording in nanoseconds.
        unix_timestamp: Start time of the recording in epoch nanoseconds.
        topics: List of topics recorded in the bag.
        message_count: Total number of messages recorded.
        db_path: Relative path to the underlying database file.
    """

    duration: float
    start_time: float
    topics: List[Topic]
    message_count: int
    db_path: str


class BagRecordingRequestSchema(BaseModel):
    """Request schema for creating a new rosbag recording.

    Attributes:
        name: Base name to use for the bag.
        topics: Optional list of topic names to record. If None, all topics are recorded.
        detail: Description or purpose of the recording.
        tags: Optional list of tags to classify or label the recording.
    """

    name: str
    topics: List[str]
    detail: str
    tags: Optional[List[str]]
