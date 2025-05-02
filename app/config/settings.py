import os
import dotenv
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
from app.logging import logger


def get_settings_file_name():
    dotenv.load_dotenv()
    name = os.getenv("API_SETTINGS_NAME")
    if name:
        if name in os.listdir("./config"):
            logger.info(f"Loading custom API settings from config/{name}")
            return f"config/{name}"
    logger.info(f"Loading default API settings from config/settings.yaml")
    return "config/settings.yaml"


SETTINGS_FILE = os.getenv("API_SETTINGS_NAME")


class AppSettings(BaseSettings):
    api: APISettings = APISettings()
    managers: ManagersSettings = ManagersSettings()
    device: DeviceSettings = DeviceSettings()

    model_config = SettingsConfigDict(yaml_file=get_settings_file_name())

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
