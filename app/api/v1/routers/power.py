from fastapi import APIRouter
from app.core.exceptions import NotYetImplementedException


router = APIRouter(prefix="/api/v1/computer")


@router.post(
    "/power/poweroff",
    description="Request to power off the onboard computer. Currently not implemented.",
    tags=["computer"],
)
def poweroff_computer():
    raise NotYetImplementedException(
        "Endpoint /power/poweroff has not yet been implemented"
    )


@router.post(
    "/power/reboot",
    description="Request to reboot the onboard computer. Currently not implemented.",
    tags=["computer"],
)
def reboot_computer():
    raise NotYetImplementedException(
        "Endpoint /power/reboot has not yet been implemented"
    )
