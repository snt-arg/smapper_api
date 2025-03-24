from pydantic import BaseModel
from typing import Optional, List


class ServiceStateSchema(BaseModel):
    state: str
    value: int


class ServiceSchema(BaseModel):
    name: str
    id: str
    srv_type: str
    cmd: str
    cwd: Optional[str] = None
    env: Optional[List[str]] = None


class RosServiceSchema(BaseModel):
    name: str
    id: str
    srv_type: str
    env: Optional[List[str]] = None
    exec_type: str
    ros_distro: str
    ws: str
    pkg_name: str
    exec: str
