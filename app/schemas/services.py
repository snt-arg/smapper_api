from pydantic import BaseModel
from typing import Optional, List


class ServiceSchema(BaseModel):
    name: str
    id: str
    type: str
    cmd: str
    env: Optional[List[str]] = None


class RosServiceSchema(BaseModel):
    name: str
    id: str
    type: str
    env: Optional[List[str]] = None
    exec_type: str
    ros_distro: str
    ws: str
    pkg_name: str
    exec: str
