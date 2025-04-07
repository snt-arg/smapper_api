from fastapi import APIRouter
from app.core.exceptions import NotYetImplementedException


router = APIRouter(prefix="/api/v1/settings")


@router.post(
    "/",
    description="Get a list of available settings",
    tags=["settings"],
)
def get_settings():
    raise NotYetImplementedException("Endpoint /settings has not yet been implemented")
