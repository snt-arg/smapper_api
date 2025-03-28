from pydantic import BaseModel


class SensorSchema(BaseModel):
    """Schema representing metadata for a hardware sensor.

    Attributes:
        name: Human-readable name of the sensor (e.g., "Lidar Front").
        model: Model identifier of the sensor (e.g., "Velodyne VLP-16").
        type: Type or category of the sensor (e.g., "lidar", "camera", "imu").
        service_id: ID of the service associated with this sensor (used for control or monitoring).
    """

    name: str
    model: str
    type: str
    service_id: str
