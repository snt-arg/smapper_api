from fastapi import APIRouter


router = APIRouter(prefix="/api/v1")


@router.get("/bags")
def get_sensors():
    return ["smapper_bag1", "smapper_bag2", "smapper_bag3"]


@router.get("/bags/{bag_id}")
def get_bag(bag_id: str):
    return {"bag_id": bag_id}
