from functools import lru_cache
from typing import Annotated
from fastapi import APIRouter, Depends, HTTPException
from app.config.settings import DeviceSettings
from app.dependencies import get_device_settings


router = APIRouter(prefix="/api/v1")


@router.get("/sensors", description="Get a list of available sensors")
def get_sensors(config: Annotated[DeviceSettings, Depends(get_device_settings)]):
    return config.sensors


@router.get("/sensors/{sensor_name}", description="Get metadata of given sensor")
def get_sensor(
    sensor_name: str, config: Annotated[DeviceSettings, Depends(get_device_settings)]
):
    for sensor in config.sensors:
        if sensor.name == sensor_name:
            return sensor

    raise HTTPException(404, detail=f"Sensor {sensor_name} not found")
