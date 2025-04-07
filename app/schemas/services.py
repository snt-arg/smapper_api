from pydantic import BaseModel
from typing import Optional, Dict


class ServiceMetadataBase(BaseModel):
    """Schema for defining a basic external service.

    Attributes:
        id: Unique identifier for the service.
        name: Human-readable name of the service.
        state: The current state of the service
    """

    id: str
    name: str


class ServiceFailure(BaseModel):
    """Data model capturing details of a service failure.

    Attributes:
        ret_code: The return code from the failed process.
        std_err: The standard error output from the process.
    """

    ret_code: int
    std_err: str


class ServiceStatus(ServiceMetadataBase):
    state: str
    failure: Optional[ServiceFailure]


class ServiceConfigSchema(ServiceMetadataBase):
    """Schema for defining a basic external service.

    Attributes:
        name: Human-readable name of the service.
        id: Unique identifier for the service.
        cmd: Command to run the service.
        cwd: Optional working directory to launch the service from.
        env: Optional dictionary of environment variables to set for the service process.
    """

    cmd: str
    auto_start: bool
    restart_on_failure: bool
    cwd: Optional[str] = None
    env: Optional[Dict[str, str]] = None


class RosServiceConfigSchema(ServiceMetadataBase):
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

    auto_start: bool
    restart_on_failure: bool
    exec_type: str
    ros_distro: str
    ws: str
    pkg_name: str
    exec: str
    env: Optional[Dict[str, str]] = None
