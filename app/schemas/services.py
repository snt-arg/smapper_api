from pydantic import BaseModel
from typing import Optional, Dict


class ServiceStateSchema(BaseModel):
    """Schema representing the state of a service.

    Attributes:
        state: The current state of the service (e.g., "ACTIVE", "INACTIVE", "FAILURE").
    """

    state: str


class ServiceSchema(BaseModel):
    """Schema for defining a basic external service.

    Attributes:
        name: Human-readable name of the service.
        id: Unique identifier for the service.
        cmd: Command to run the service.
        cwd: Optional working directory to launch the service from.
        env: Optional dictionary of environment variables to set for the service process.
    """

    name: str
    id: str
    cmd: str
    cwd: Optional[str] = None
    env: Optional[Dict[str, str]] = None


class RosServiceSchema(BaseModel):
    """Schema for defining a ROS-specific service.

    Attributes:
        name: Human-readable name of the ROS service.
        id: Unique identifier for the ROS service.
        env: Optional dictionary of environment variables to set for the process.
        exec_type: Type of ROS execution ("NODE" or "LAUNCH").
        ros_distro: The ROS distribution to use (e.g., "humble", "foxy").
        ws: Path to the ROS workspace.
        pkg_name: Name of the ROS package containing the node or launch file.
        exec: The node or launch file to execute.
    """

    name: str
    id: str
    env: Optional[Dict[str, str]] = None
    exec_type: str
    ros_distro: str
    ws: str
    pkg_name: str
    exec: str
