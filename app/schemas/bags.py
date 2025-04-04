from typing import List, Optional
from pydantic import BaseModel
from .ros import TopicSchema


class RecordingStatus(BaseModel):
    status: str


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
    unix_timestamp: float
    topics: List[TopicSchema]
    message_count: int
    db_path: str


class BagCreationResponse(BaseModel):
    """Response model returned after initiating a bag recording.

    Attributes:
        bag_id: Unique identifier of the created bag.
        service_state: Current state of the rosbag service (e.g., ACTIVE, FAILURE).
    """

    bag_id: str
    service_state: str


class BagSchema(BaseModel):
    """Full representation of a recorded bag and its metadata.

    Attributes:
        id: Unique identifier for the bag.
        name: Human-readable name for the bag.
        bag_size: Size of the bag file (e.g., '1.25 GB').
        rosbag_metadata: Minimal rosbag metadata details.
        detail: Optional description or annotation about the bag.
        tags: Optional list of tags associated with the bag.
    """

    id: str
    name: str
    bag_size: str
    rosbag_metadata: MinimalRosbagMetadata
    detail: Optional[str] = ""
    tags: Optional[List[str]] = []


class BagRecordingRequestSchema(BaseModel):
    """Request schema for creating a new rosbag recording.

    Attributes:
        name: Base name to use for the bag.
        topics: Optional list of topic names to record. If None, all topics are recorded.
        detail: Description or purpose of the recording.
        tags: Optional list of tags to classify or label the recording.
    """

    name: str
    topics: Optional[List[str]]
    detail: str
    tags: Optional[List[str]]


class RosbagMetadata(BagRecordingRequestSchema):
    """Extended recording metadata including the unique bag ID.

    Inherits from:
        BagRecordingRequestSchema

    Attributes:
        id: Unique identifier assigned to the bag.
    """

    id: str
