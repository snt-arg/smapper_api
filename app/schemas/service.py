from pydantic import BaseModel, Field
from typing import Optional, Dict


class ServiceMetadataBase(BaseModel):
    """Basic schema for an external service."""

    id: str = Field(description="Unique identifier for the service")
    name: str = Field(description="Human-readable name of the service")


class ServiceFailure(BaseModel):
    """Data model capturing details of a service failure."""

    ret_code: int = Field(description="The return code from the failed process")
    std_err: str = Field(description="The standard error output from the process")


class ServiceStatus(ServiceMetadataBase):
    """Schema representing the status of an external service."""

    state: str = Field(description="The current state of the service")
    failure: Optional[ServiceFailure] = Field(
        default=None, description="Details of a service failure, if any"
    )


class ServiceConfigSchema(ServiceMetadataBase):
    """Schema for configuring an external service."""

    cmd: str = Field(description="Command to run the service")
    auto_start: Optional[bool] = Field(
        default=False,
        description="Flag indicating if the service should start automatically",
    )
    restart_on_failure: Optional[bool] = Field(
        default=False,
        description="Flag indicating if the service should restart on failure",
    )
    cwd: Optional[str] = Field(
        default=None,
        description="Optional working directory to launch the service from",
    )
    env: Optional[Dict[str, str]] = Field(
        default=None,
        description="Optional dictionary of environment variables for the service process",
    )


class RosServiceConfigSchema(ServiceMetadataBase):
    """Schema for configuring a ROS-specific service."""

    auto_start: Optional[bool] = Field(
        default=False,
        description="Flag indicating if the ROS service should start automatically",
    )
    restart_on_failure: Optional[bool] = Field(
        default=False,
        description="Flag indicating if the ROS service should restart on failure",
    )
    exec_type: str = Field(description="Type of ROS execution ('NODE' or 'LAUNCH')")
    ros_distro: str = Field(
        description="The ROS distribution to use (e.g., 'humble', 'foxy')"
    )
    ws: str = Field(description="Path to the ROS workspace")
    pkg_name: str = Field(
        description="Name of the ROS package containing the node or launch file"
    )
    exec: str = Field(description="The node or launch file to execute")
    env: Optional[Dict[str, str]] = Field(
        None, description="Optional dictionary of environment variables for the process"
    )
