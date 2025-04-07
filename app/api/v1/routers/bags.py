from typing import Annotated, List
from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from app.db.database import get_db
from app.schemas.ros.rosbag import RosbagMetadata, RosbagMetadataUpdate
from app.crud import rosbag as crud


router = APIRouter(prefix="/api/v1/rosbags")


@router.get(
    "/",
    description="Return a list of all available bag recordings with associated metadata.",
    response_model=List[RosbagMetadata],
    tags=["rosbags"],
)
def read_bags(db: Annotated[Session, Depends(get_db)]):
    return crud.get_rosbags(db)


@router.get(
    "/{rosbag_id}",
    response_model=RosbagMetadata,
    tags=["rosbags"],
)
def read_rosbag(rosbag_id: int, db: Annotated[Session, Depends(get_db)]):
    rosbag = crud.get_rosbag(db, rosbag_id)
    if not rosbag:
        raise HTTPException(status_code=404, detail="Rosbag not found")
    return rosbag


@router.put(
    "/{rosbag_id}",
    response_model=RosbagMetadata,
    tags=["rosbags"],
)
def update_rosbag_route(
    rosbag_id: int,
    update_data: RosbagMetadataUpdate,
    db: Annotated[Session, Depends(get_db)],
):
    updated = crud.update_rosbag(db, rosbag_id, update_data)
    if not updated:
        raise HTTPException(status_code=404, detail="Rosbag not found")
    return updated


@router.delete(
    "/{rosbag_id}",
    tags=["rosbags"],
)
def delete_rosbag_route(rosbag_id: int, db: Annotated[Session, Depends(get_db)]):
    deleted = crud.delete_rosbag(db, rosbag_id)
    if not deleted:
        raise HTTPException(status_code=404, detail="Rosbag not found")
    return {"message": "Deleted"}
