from typing import List
from fastapi import APIRouter
from app.exceptions import NotYetImplemented
from app.schemas import BagSchema


router = APIRouter(prefix="/api/v1")


@router.get("/bags")
def get_sensors() -> List[BagSchema]:
    raise NotYetImplemented("Endpoint /bags has not yet been implemented")


@router.get("/bags/{id}")
def get_bag(id: str) -> BagSchema:
    raise NotYetImplemented("Endpoint /bag/{id} has not yet been implemented")
