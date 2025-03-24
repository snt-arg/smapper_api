from fastapi import APIRouter
from app.exceptions import NotYetImplementedException


router = APIRouter(prefix="/api/v1")


@router.post("/power/poweroff")
def poweroff_computer():
    raise NotYetImplementedException(
        "Endpoint /power/poweroff has not yet been implemented"
    )


@router.post("/power/reboot")
def reboot_computer():
    raise NotYetImplementedException(
        "Endpoint /power/reboot has not yet been implemented"
    )
