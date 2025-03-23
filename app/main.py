from contextlib import asynccontextmanager
from logging import log
from typing import Annotated

from fastapi import Depends, FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from app.core.services.service import ServiceException
from app.routers.v1 import sensors_router, bags_router, power_router, services_router
from app.core.service_manager import ServiceManager, ServiceManagerException
from app.dependencies import (
    get_api_settings,
    get_device_settings,
    get_service_manager,
)
from app.logger import logger
from app.config.settings import DeviceSettings
from app.schemas.services import Service, RosService


def setup_services(
    service_manager: Annotated[ServiceManager, Depends(get_service_manager)],
    config: Annotated[DeviceSettings, Depends(get_device_settings)],
) -> None:
    logger.info("Setting up services")
    for service in config.services:
        if isinstance(service, Service):
            service_manager.add_service(service.id, service.name, cmd=service.cmd)
        elif isinstance(service, RosService):
            service_manager.add_service(service.id, service.name, cmd=service.exec)


def stop_services(
    service_manager: Annotated[ServiceManager, Depends(get_service_manager)],
) -> None:
    service_manager.stop_all()


@asynccontextmanager
async def lifespan(
    app: FastAPI,
):
    # Executed on startup
    service_manager = get_service_manager()
    config = get_device_settings()

    setup_services(service_manager, config)

    yield
    # Executed on shutdown
    stop_services(service_manager)


api_settings = get_api_settings()

if api_settings is None:
    exit(1)


app = FastAPI(
    title=api_settings.title,
    summary=api_settings.summary,
    description=api_settings.description,
    version=api_settings.version,
    docs_url=api_settings.docs_url,
    openapi_url=api_settings.openapi_url,
    debug=api_settings.debug,
    lifespan=lifespan,
)


app.include_router(sensors_router)
app.include_router(services_router)
app.include_router(bags_router)
app.include_router(power_router)

# INFO: CORS Middleware origins.
# Needed if frontend server is running on different port for instance.
app.add_middleware(
    CORSMiddleware,
    allow_origins=api_settings.allowed_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.exception_handler(ServiceManagerException)
async def service_manager_exception_handler(
    request: Request, exc: ServiceManagerException
):
    return JSONResponse(
        status_code=500,
        content={"message": f"{exc}"},
    )


@app.exception_handler(ServiceException)
async def service_exception_handler(request: Request, exc: ServiceException):
    return JSONResponse(
        status_code=500,
        content={"message": f"{exc}"},
    )
