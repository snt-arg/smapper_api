from typing import Tuple, Type

from pydantic_settings import (
    BaseSettings,
    PydanticBaseSettingsSource,
    SettingsConfigDict,
    YamlConfigSettingsSource,
)

from app.config.managers_settings import ManagersSettings
from app.config.device_settings import DeviceSettings
from app.config.api_settings import APISettings


class AppSettings(BaseSettings):
    api: APISettings = APISettings()
    managers: ManagersSettings = ManagersSettings()
    device: DeviceSettings = DeviceSettings()

    model_config = SettingsConfigDict(yaml_file="config/settings.yaml")

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
