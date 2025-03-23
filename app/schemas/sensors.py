from pydantic import BaseModel


class Sensor(BaseModel):
    name: str
    model: str
    type: str
    service_id: str
