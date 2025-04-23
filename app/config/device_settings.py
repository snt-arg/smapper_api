from pydantic import Field
from pydantic_settings import BaseSettings

from app.schemas import (
    SensorMetadataBase,
    OnboardPCSchema,
)


class DeviceSettings(BaseSettings):
    revision: str = Field(default="1.0")
    name: str = Field(default="smapper")
    sensors: list[SensorMetadataBase] = Field(default=[])
    onboard_pc: OnboardPCSchema = Field(
        default=OnboardPCSchema(model="Jetson AGX Orin Development Kit")
    )
