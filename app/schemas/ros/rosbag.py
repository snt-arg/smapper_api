from pydantic import BaseModel, Field
from typing import List, Optional

from pydantic_settings import SettingsConfigDict

from app.schemas.ros.topic import TopicBase


class RosbagTopic(TopicBase):
    """Extended topic model including a unique identifier for database."""

    id: int = Field(description="Unique identifier for database")
    model_config = SettingsConfigDict(from_attributes=True)


class RosbagMetadataBase(BaseModel):
    """Base model representing core metadata of a ROS bag."""

    name: str = Field(description="The name of the ROS bag")
    size: Optional[int] = Field(
        default=None, description="The size of the ROS bag file in bytes, if available"
    )
    duration: Optional[float] = Field(
        default=None,
        description="The duration of the bag recording (in seconds or another defined unit)",
    )
    start_time: Optional[float] = Field(
        default=None, description="The start time of the recording as a Unix timestamp"
    )
    end_time: Optional[float] = Field(
        default=None, description="The end time of the recording as a Unix timestamp"
    )
    detail: Optional[str] = Field(
        default=None, description="Additional details or description of the ROS bag"
    )
    tags: Optional[str] = Field(
        default=None, description="Comma-separated tags or keywords for the ROS bag"
    )


class RosbagMetadataCreate(RosbagMetadataBase):
    """Model for creating a new ROS bag metadata record."""

    topics: List[TopicBase] = Field(
        default_factory=list, description="A list of topics associated with the ROS bag"
    )
    rosbag_path: str = Field(description="The file system path to the ROS bag file")


class RosbagMetadataUpdate(BaseModel):
    """Model for updating an existing ROS bag metadata record."""

    name: Optional[str] = Field(default=None, description="New name for the ROS bag")
    detail: Optional[str] = Field(
        default=None, description="Updated details for the ROS bag"
    )
    tags: Optional[List[str]] = Field(
        default=None, description="Updated comma-separated tags for the ROS bag"
    )


class RosbagMetadata(RosbagMetadataBase):
    """Model representing complete ROS bag metadata including a unique identifier and topics."""

    id: int = Field(description="Unique identifier for the ROS bag metadata record")
    topics: List[RosbagTopic] = Field(
        default_factory=list, description="List of topics recorded in the ROS bag"
    )
    model_config = SettingsConfigDict(from_attributes=True)


class RosbagFileMetadata(BaseModel):
    """Minimal model representing metadata for a ROS2 bag file."""

    duration: float = Field(description="Duration of the recording in nanoseconds")
    start_time: float = Field(
        description="Start time of the recording as a Unix timestamp in nanoseconds"
    )
    topics: List[TopicBase] = Field(description="List of topics recorded in the bag")
    message_count: int = Field(description="Total number of messages recorded")
    db_path: str = Field(description="Relative path to the underlying database file")
