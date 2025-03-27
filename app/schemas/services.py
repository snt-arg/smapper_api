from pydantic import BaseModel
from typing import Optional, List, Dict


class ServiceStateSchema(BaseModel):
    state: str


# TODO: remove srv_type, and instead just rely on pydantic deserialization
class ServiceSchema(BaseModel):
    name: str
    id: str
    cmd: str
    cwd: Optional[str] = None
    env: Optional[Dict[str, str]] = None


class RosServiceSchema(BaseModel):
    name: str
    id: str
    env: Optional[Dict[str, str]] = None
    exec_type: str
    ros_distro: str
    ws: str
    pkg_name: str
    exec: str
