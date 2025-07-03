from typing import Optional, List
from pydantic import BaseModel, Field


class RecordingMetadata(BaseModel):
    """Model representing metadata for a rosbag recording."""

    bag_name: str = Field(description="The base name of the bag")
    start_time: float = Field(
        description="The start time of the recording as a Unix timestamp"
    )
    elapsed_time: float = Field(description="Elapsed time of the recording in seconds")
    stop_time: Optional[float] = Field(
        default=None,
        description="The stop time of the recording as a Unix timestamp, if available",
    )
    bag_size: int = Field(description="The size of the bag file in bytes")
    topics: List[str] = Field(description="List of topic names recorded")


class RecordingStatus(BaseModel):
    """Model representing the current state of a recording."""

    state: str = Field(description="Current state of the recording")
    metadata: Optional[RecordingMetadata] = Field(
        default=None, description="Metadata information for the recording"
    )

class BagRecorderCompressionSettings(BaseModel):
    enabled: bool = Field(default=False)
    format: str = Field(default="zstd")
    mode: str = Field(default="file")



class RecordingStartRequest(BaseModel):
    """Request schema for starting a new rosbag recording."""

    name: str = Field(description="Base name to use for the bag")
    topics: List[str] = Field(
        description="List of topic names to record; if empty, all topics are recorded"
    )
    detail: str = Field(description="Description or purpose of the recording")
    tags: Optional[List[str]] = Field(
        default=None,
        description="Optional list of tags to classify or label the recording",
    )
    compression: Optional[BagRecorderCompressionSettings] = Field(
        default=BagRecorderCompressionSettings,
        description="Compression settings to be used by ros2 bag recorder"
    )
