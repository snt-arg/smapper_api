from fastapi import APIRouter


router = APIRouter(prefix="/api/v1")


@router.post("/power/poweroff")
def poweroff_computer():
    return


@router.post("/power/reboot")
def reboot_computer():
    return
