from pydantic import BaseModel


class SensorSchema(BaseModel):
    name: str
    model: str
    type: str
    service_id: str
