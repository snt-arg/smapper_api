from typing import Annotated, List
from fastapi import APIRouter, Depends, HTTPException
from app.core.bag_manager import BagManager
from app.dependencies import get_bag_manager
from app.exceptions import BagNotFoundException, NotYetImplementedException
from app.schemas import BagSchema


router = APIRouter(prefix="/api/v1")


@router.get("/bags")
def get_bags(
    bag_manager: Annotated[BagManager, Depends(get_bag_manager)],
) -> List[BagSchema]:
    return bag_manager.get_bags()


@router.get("/bags/{id}")
def get_bag_by_id(
    id: str, bag_manager: Annotated[BagManager, Depends(get_bag_manager)]
) -> BagSchema:
    try:
        bag = bag_manager.get_bag_by_id(id)
    except BagNotFoundException as e:
        raise HTTPException(404, e.detail)

    return bag
