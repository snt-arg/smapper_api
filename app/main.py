from functools import lru_cache
from contextlib import asynccontextmanager
import logging
from typing import Annotated

from fastapi import Depends, FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse

from app.routers import sensors
from app.config.settings import APISettings
from app.routers.v1 import sensors_router, bags_router, power_router, services_router
from app.core.service_manager import get_service_manager
from app.config.config import Configuration, RosService, Service

logger = logging.getLogger("unvicorn.error")

# To fix issue with CORS when requesting on the browser
origins = [
    "http://localhost",
    "http://localhost:8080",
    "http://localhost:5173",
]


@asynccontextmanager
async def lifespan(app: FastAPI):
    # read configuration
    # setup services
    # start services if autostart is true
    manager = get_service_manager()

    logger.info("Loading configuration")
    config = Configuration.load("app/config/device.yaml")

    for service in config.services:
        if isinstance(service, Service):
            manager.add_service(service.id, service.name, cmd=service.cmd)
        elif isinstance(service, RosService):
            manager.add_service(service.id, service.name, cmd=service.exec)

    logger.info("Before Start FASTAPI")
    yield
    logger.info("After End FASTAPI")


@lru_cache
def get_settings():
    return APISettings()  # type: ignore


app = FastAPI(
    debug=False,
    lifespan=lifespan,
)


app.include_router(sensors_router)
app.include_router(services_router)
app.include_router(bags_router)
app.include_router(power_router)

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/")
async def info(settings: Annotated[APISettings, Depends(get_settings)]):
    return HTMLResponse(
        f"<h1>API is up and running. Version of the API: {settings.version}</h1>"
    )
