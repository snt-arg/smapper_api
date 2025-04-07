from typing import List, Optional
from pydantic import BaseModel


class RecordingMetadata(BaseModel):
    bag_name: str
    start_time: float
    elapsed_time: float
    stop_time: Optional[float] = None
    bag_size: int
    topics: List[str]


class RecordingStatus(BaseModel):
    state: str
    metadata: Optional[RecordingMetadata] = None


class RecordingStartRequest(BaseModel):
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
