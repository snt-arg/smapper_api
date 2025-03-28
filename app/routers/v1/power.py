from fastapi import APIRouter
from app.exceptions import NotYetImplementedException


router = APIRouter(prefix="/api/v1")


@router.post(
    "/power/poweroff",
    description="Request to power off the onboard computer. Currently not implemented.",
)
def poweroff_computer():
    raise NotYetImplementedException(
        "Endpoint /power/poweroff has not yet been implemented"
    )


@router.post(
    "/power/reboot",
    description="Request to reboot the onboard computer. Currently not implemented.",
)
def reboot_computer():
    raise NotYetImplementedException(
        "Endpoint /power/reboot has not yet been implemented"
    )
