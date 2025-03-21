from functools import lru_cache
from typing import Annotated
from fastapi import APIRouter, Depends, HTTPException
from app.config.config import Configuration


router = APIRouter(prefix="/api/v1")


@lru_cache()
def get_configuration():
    return Configuration.load("app/config/device.yaml")


@router.get("/sensors", description="Get a list of available sensors")
def get_sensors(config: Annotated[Configuration, Depends(get_configuration)]):
    return config.sensors


@router.get("/sensors/{sensor_name}", description="Get metadata of given sensor")
def get_sensor(
    sensor_name: str, config: Annotated[Configuration, Depends(get_configuration)]
):
    for sensor in config.sensors:
        if sensor.name == sensor_name:
            return sensor

    raise HTTPException(404, detail=f"Sensor with name <{sensor_name}> does not exist")
