from fastapi import APIRouter
from app.exceptions import NotYetImplemented


router = APIRouter(prefix="/api/v1")


@router.post("/power/poweroff")
def poweroff_computer():
    raise NotYetImplemented("Endpoint /power/poweroff has not yet been implemented")


@router.post("/power/reboot")
def reboot_computer():
    raise NotYetImplemented("Endpoint /power/reboot has not yet been implemented")
