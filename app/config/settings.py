from pydantic import BaseModel, Field
from typing import Any, Dict, List, Optional, Tuple, Type

from pydantic_settings import (
    BaseSettings,
    PydanticBaseSettingsSource,
    SettingsConfigDict,
    YamlConfigSettingsSource,
)

from app.schemas import (
    ServiceConfigSchema,
    RosServiceConfigSchema,
    SensorMetadataBase,
    OnboardPCSchema,
)


class APISettings(BaseSettings):
    title: str = Field(default="SMapper API")
    summary: str = Field(default="API for the SMapper handheld device")
    version: str = Field(default="0.1.0")
    description: str = Field(
        default_factory=lambda: """# SMapper API

SMapper API is the main contact point with the handheld device. It can be used 
to check which sensors are currently installed, start/stop/restart a set of services,
and more, like powering off the onboard computer. For more information, you can visit
the /docs endpoint for the device documentation
"""
    )
    docs_url: str = "/api/docs"
    openapi_url: str = "/api/openapi.json"
    debug: bool = False

    allowed_origins: list[str] = []
    openapi_tags: list[Dict[str, Any]]

    model_config = SettingsConfigDict(
        yaml_file="config/api_config.yaml", env_file=".env", env_prefix="API_"
    )

    @classmethod
    def settings_customise_sources(
        cls,
        settings_cls: Type[BaseSettings],
        init_settings: PydanticBaseSettingsSource,
        env_settings: PydanticBaseSettingsSource,
        dotenv_settings: PydanticBaseSettingsSource,
        file_secret_settings: PydanticBaseSettingsSource,
    ) -> Tuple[PydanticBaseSettingsSource, ...]:
        return (YamlConfigSettingsSource(settings_cls),)


class RosSchema(BaseModel):
    topics_to_monitor: List[str]


class DeviceSettings(BaseSettings):
    services: list[ServiceConfigSchema | RosServiceConfigSchema] = Field(default=[])
    sensors: list[SensorMetadataBase] = Field(default=[])
    onboard_pc: OnboardPCSchema = Field(
        default=OnboardPCSchema(model="Jetson AGX Orin Development Kit")
    )
    revision: str = Field(default="1.0")
    device_name: str = Field(default="smapper")
    bags_storage_path: str = Field(default="")
    ros: RosSchema = Field(default=RosSchema(topics_to_monitor=[]))

    model_config = SettingsConfigDict(yaml_file="config/device_config.yaml")

    @classmethod
    def settings_customise_sources(
        cls,
        settings_cls: Type[BaseSettings],
        init_settings: PydanticBaseSettingsSource,
        env_settings: PydanticBaseSettingsSource,
        dotenv_settings: PydanticBaseSettingsSource,
        file_secret_settings: PydanticBaseSettingsSource,
    ) -> Tuple[PydanticBaseSettingsSource, ...]:
        return (YamlConfigSettingsSource(settings_cls),)


class AppSettings(BaseSettings):
    api: APISettings = APISettings()
    device: DeviceSettings = DeviceSettings()
