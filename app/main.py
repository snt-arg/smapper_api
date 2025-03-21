from contextlib import asynccontextmanager
from typing import Annotated

from fastapi import Depends, FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from app.core.services.service import ServiceException
from app.routers.v1 import sensors_router, bags_router, power_router, services_router
from app.core.service_manager import ServiceManager, ServiceManagerException
from app.config.config import Configuration, RosService, Service
from app.dependencies import get_configuration, get_service_manager
from app.config.api_config import *
from app.logger import logger


def setup_services(
    service_manager: Annotated[ServiceManager, Depends(get_service_manager)],
    config: Annotated[Configuration, Depends(get_configuration)],
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
    config = get_configuration()

    setup_services(service_manager, config)

    yield
    # Executed on shutdown
    stop_services(service_manager)


app = FastAPI(
    title=TITLE,
    summary=SUMMARY,
    description=DESCRIPTION,
    version=VERSION,
    docs_url="/api/docs",
    lifespan=lifespan,
    debug=True,
)


app.include_router(sensors_router)
app.include_router(services_router)
app.include_router(bags_router)
app.include_router(power_router)

# INFO: CORS Middleware origins.
# Needed due to frontend server running on different port
origins = [
    "http://localhost",
    "http://localhost:5173",  # Frontend Vite server
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.exception_handler(ServiceManagerException)
async def unicorn_exception_handler(request: Request, exc: ServiceManagerException):
    return JSONResponse(
        status_code=500,
        content={"message": f"{exc}"},
    )


@app.exception_handler(ServiceException)
async def unicorn_exception_handler(request: Request, exc: ServiceException):
    return JSONResponse(
        status_code=500,
        content={"message": f"{exc}"},
    )
