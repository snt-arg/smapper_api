from pydantic import Field
from pydantic_settings import BaseSettings


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
