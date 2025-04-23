from pydantic import BaseModel, Field
from typing import Dict, List
from app.schemas import (
    ServiceConfigSchema,
    RosServiceConfigSchema,
)


class TopicMonitorSettings(BaseModel):
    topics_blacklist: List[str] = Field(default=[])


class BagRecorderSettings(BaseModel):
    storage_path: str = Field(default="")


class RosManagers(BaseModel):
    bag_recorder: BagRecorderSettings = Field(default=BagRecorderSettings())
    topic_monitor: TopicMonitorSettings = Field(default=TopicMonitorSettings())


class ManagersSettings(BaseModel):
    services: list[ServiceConfigSchema | RosServiceConfigSchema] = Field(default=[])
    ros: RosManagers = RosManagers()
