from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.db.database import init_db
from app.core.lifespan import lifespan
from app.di import get_api_settings
from app.core.exceptions import init_exception_handlers
from app.api.v1 import (
    sensors_router,
    bags_router,
    power_router,
    services_router,
    ros_router,
    recordings_router,
    settings_router,
)

init_db()

# Get settings loaded from configuration file
api_settings = get_api_settings()

app = FastAPI(
    title=api_settings.title,
    summary=api_settings.summary,
    description=api_settings.description,
    version=api_settings.version,
    docs_url=api_settings.docs_url,
    openapi_url=api_settings.openapi_url,
    debug=api_settings.debug,
    lifespan=lifespan,
    openapi_tags=[
        {"name": "services", "description": "Start/stop/monitor services"},
        {"name": "sensors", "description": "Sensor state info"},
        {"name": "recordings", "description": "Start/stop recordings"},
        {"name": "rosbags", "description": "CRUD operations on rosbags recorded"},
        {"name": "ros", "description": "Ros related endpoints"},
        {"name": "topics", "description": "Ros2 topic monitoring info"},
        {"name": "computer", "description": "Information about onboard computer"},
        {"name": "settings", "description": "Check and modify api/device settings"},
    ],
)


# Add API routers
app.include_router(sensors_router)
app.include_router(services_router)
app.include_router(bags_router)
app.include_router(power_router)
app.include_router(ros_router)
app.include_router(recordings_router)
app.include_router(settings_router)

# Setup CORS Middleware.
app.add_middleware(
    CORSMiddleware,
    allow_origins=api_settings.allowed_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

init_exception_handlers(app)
