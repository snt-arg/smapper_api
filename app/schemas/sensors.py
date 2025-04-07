from pydantic import BaseModel, Field


class SensorMetadataBase(BaseModel):
    """Schema representing metadata for a hardware sensor."""

    name: str = Field(
        description="Human-readable name of the sensor (e.g., 'Lidar Front')"
    )
    model: str = Field(
        description="Model identifier of the sensor (e.g., 'Velodyne VLP-16')"
    )
    type: str = Field(
        description="Type or category of the sensor (e.g., 'lidar', 'camera', 'imu')"
    )
    service_id: str = Field(
        description="ID of the service associated with this sensor (used for control or monitoring)"
    )
