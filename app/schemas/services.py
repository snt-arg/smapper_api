from pydantic import BaseModel
from typing import Optional, List, Dict


class ServiceStateSchema(BaseModel):
    state: str
    value: int


class ServiceSchema(BaseModel):
    name: str
    id: str
    srv_type: str
    cmd: str
    cwd: Optional[str] = None
    env: Optional[Dict[str, str]] = None


class RosServiceSchema(BaseModel):
    name: str
    id: str
    srv_type: str
    env: Optional[Dict[str, str]] = None
    exec_type: str
    ros_distro: str
    ws: str
    pkg_name: str
    exec: str
