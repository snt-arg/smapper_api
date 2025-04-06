import shutil
from typing import List
from sqlalchemy.orm import Session
from app.models.rosbag_metadata import RosbagMetadata
from app.models.rosbag_topic import RosbagTopic
from app.schemas.rosbag import RosbagMetadataCreate


def create_rosbag(db: Session, rosbag: RosbagMetadataCreate) -> RosbagMetadata:
    db_rosbag = RosbagMetadata(**rosbag.model_dump(exclude={"topics"}))
    db.add(db_rosbag)
    db.commit()
    db.refresh(db_rosbag)

    for topic in rosbag.topics:
        db_topic = RosbagTopic(**topic.model_dump(), rosbag_id=db_rosbag.id)
        db.add(db_topic)

    db.commit()
    db.refresh(db_rosbag)
    return db_rosbag


def get_rosbags(db: Session, skip: int = 0, limit: int = 10) -> List[RosbagMetadata]:
    return db.query(RosbagMetadata).offset(skip).limit(limit).all()


def get_rosbag(db: Session, rosbag_id: int) -> RosbagMetadata | None:
    return db.query(RosbagMetadata).filter(RosbagMetadata.id == rosbag_id).first()


def update_rosbag(
    db: Session, rosbag_id: int, update_data: dict
) -> RosbagMetadata | None:
    rosbag = get_rosbag(db, rosbag_id)
    if not rosbag:
        return None
    for key, value in update_data.items():
        setattr(rosbag, key, value)
    db.commit()
    db.refresh(rosbag)
    return rosbag


def delete_rosbag(db: Session, rosbag_id: int) -> RosbagMetadata | None:
    rosbag = get_rosbag(db, rosbag_id)
    if not rosbag:
        return None
    db.delete(rosbag)
    db.commit()

    shutil.rmtree(f"{rosbag.rosbag_path}")
    return rosbag
