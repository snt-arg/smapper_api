from pydantic import BaseModel, Field
from typing import Dict, List, Optional
from app.schemas import (
    ServiceConfigSchema,
    RosServiceConfigSchema,
)


class TopicMonitorSettings(BaseModel):
    topics_blacklist: List[str] = Field(default=[])
    monitor_rate: float = Field(default=2)
    discover_rate: float = Field(default=3)
    idle_timeout: float = Field(default=5)


class BagRecorderSettings(BaseModel):
    storage_dir: str = Field(default="")
    ws: Optional[str] = Field(None, description="Path to the ROS workspace")
    env: Optional[Dict[str, str]] = Field(
        default=None,
        description="Optional dictionary of environment variables for the service process",
    )


class RosManagers(BaseModel):
    bag_recorder: BagRecorderSettings = Field(default=BagRecorderSettings())
    topic_monitor: TopicMonitorSettings = Field(default=TopicMonitorSettings())


class ManagersSettings(BaseModel):
    services: list[ServiceConfigSchema | RosServiceConfigSchema] = Field(default=[])
    ros: RosManagers = RosManagers()
