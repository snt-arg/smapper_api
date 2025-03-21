from pydantic import BaseModel, config, dataclasses
from typing import Optional, List
import logging
from typing import Any
import yaml

logger = logging.getLogger("uvicorn")


class OnboardPC(BaseModel):
    model: str


class Sensor(BaseModel):
    name: str
    model: str
    type: str
    service_id: str


class Service(BaseModel):
    name: str
    id: str
    type: str
    cmd: str
    env: Optional[List[str]] = None


class RosService(BaseModel):
    name: str
    id: str
    type: str
    env: Optional[List[str]] = None
    exec_type: str
    ros_distro: str
    ws: str
    pkg_name: str
    exec: str


class Configuration(BaseModel):
    services: list[Service | RosService]
    sensors: list[Sensor]
    onboard_pc: OnboardPC
    autostart_services: bool
    revision: str
    device_name: str


def load_config(path: str) -> Configuration:
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    return Configuration.model_validate(data)
