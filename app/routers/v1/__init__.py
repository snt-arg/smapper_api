from .sensors import router as sensors_router
from .bags import router as bags_router
from .power import router as power_router
from .services import router as services_router

__all__ = ["sensors_router", "bags_router", "power_router", "services_router"]
