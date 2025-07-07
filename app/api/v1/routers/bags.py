import os
import time
import zipstream
from typing import Annotated, List

from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import StreamingResponse
from sqlalchemy.orm import Session

from app.crud import rosbag as crud
from app.db.database import get_db
from app.di import get_api_settings
from app.schemas.ros.rosbag import RosbagMetadata, RosbagMetadataUpdate
from app.utils.files import calculate_total_size

router = APIRouter(prefix="/api/v1/rosbags")


@router.get(
    "/",
    description="Return a list of all available bag recordings with associated metadata.",
    response_model=List[RosbagMetadata],
    tags=["rosbags"],
)
def read_bags(db: Annotated[Session, Depends(get_db)]):
    return crud.get_rosbags(db, limit=None)


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


@router.get(
    "/{rosbag_id}/download",
    tags=["rosbags"],
)
async def download_bag(rosbag_id: int, db: Annotated[Session, Depends(get_db)]):
    bag = crud.get_rosbag(db, rosbag_id)

    if not bag:
        raise HTTPException(status_code=404, detail="Rosbag not found")

    bag_path = str(bag.rosbag_path)

    # USE compression = ZIP_DEFLATED for applying compression (can lead to slow speeds)
    content_stream = zipstream.ZipFile(mode="w", compression=zipstream.ZIP_STORED, allowZip64=True)

    # Setting minimum epoch time for ZIP to work
    safe_epoch = time.mktime((1980, 1, 1, 0, 0, 0, 0, 0, 0))

    # Walk the rosbag files, metadata & db and prepare them for streaming
    for root, _, files in os.walk(bag_path):
        for file in files:
            full_path = os.path.join(root, file)
            arcname = os.path.relpath(full_path, start=bag_path)

            # Get current modified time
            stat = os.stat(full_path)
            if stat.st_mtime < safe_epoch:
                # Overwrite with a safe timestamp
                os.utime(full_path, (safe_epoch, safe_epoch))

            content_stream.write(
                full_path, arcname
            )  # this queues the file for streaming

    # INFO: Disabling total_size since it will fail as the size is not the same as 
    # the final transmitted file
    # total_size = calculate_total_size(bag_path)
    return StreamingResponse(
        content_stream,  # type: ignore
        media_type="application/zip",
        headers={
            "Content-Disposition": f'attachment; filename="{bag.name}.zip"',
             # "Content-Length": str(total_size)
        }
    )
