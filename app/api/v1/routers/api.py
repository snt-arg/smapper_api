from typing import Annotated, Dict
from app.config.api_settings import APISettings
from fastapi import APIRouter, Depends
from app.di import get_api_settings

router = APIRouter(prefix="/api/v1")


@router.get(
    "/version",
    description="Retrieve the current version of the API",
    tags=["api"],
)
def poweroff_computer(
    config: Annotated[APISettings, Depends(get_api_settings)],
) -> Dict[str, str]:
    return {"version": config.version}
