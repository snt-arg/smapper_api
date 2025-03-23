from pydantic import Field
from typing import Tuple, Type

from pydantic_settings import (
    BaseSettings,
    PydanticBaseSettingsSource,
    SettingsConfigDict,
    YamlConfigSettingsSource,
)

from app.schemas.sensors import Sensor
from app.schemas.services import Service, RosService
from app.schemas.onboard_pc import OnboardPC


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


class DeviceSettings(BaseSettings):
    services: list[Service | RosService] = Field(default=[])
    sensors: list[Sensor] = Field(default=[])
    onboard_pc: OnboardPC = Field(
        default=OnboardPC(model="Jetson AGX Orin Development Kit")
    )
    revision: str = Field(default="1.0")
    device_name: str = Field(default="smapper")
    autostart_services: bool = Field(default=False)

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
