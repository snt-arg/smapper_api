from fastapi import APIRouter


router = APIRouter(prefix="/api/v1")


@router.get("/sensors", description="Get a list of available sensors")
def get_sensors():
    return ["lidar", "argus_cameras", "realsense"]


@router.get("/sensors/{sensor_name}", description="Get metadata of given sensor")
def get_sensor(sensor_name: str):
    # Get actual sensor metadata
    metadata = {
        "name": "lidar",
        "type": "3d lidar",
        "model": "Ouster OS0",
        "service": "ACTIVE",
    }

    return metadata


@router.post("/sensors/{sensor_name}/start", description="Start sensor service")
def start_sensor(sensor_name: str):
    # Start sensor service with ProcessPool
    return {"status": "OK"}


@router.post("/sensors/{sensor_name}/stop", description="Stop sensor service")
def stop_sensor(sensor_name: str):
    # Stop sensor service with ProcessPool
    return {"status": "OK"}
