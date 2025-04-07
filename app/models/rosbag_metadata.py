from sqlalchemy import Column, Integer, String, Text
from sqlalchemy.orm import relationship
from app.db.database import Base


class RosbagMetadata(Base):
    __tablename__ = "rosbag_metadata"

    id = Column(Integer, primary_key=True, index=True)
    name = Column(String, nullable=False)
    size = Column(Integer)
    duration = Column(Integer)
    start_time = Column(Integer)
    end_time = Column(Integer)
    detail = Column(Text)
    tags = Column(Text)
    rosbag_path = Column(String, nullable=False)

    topics = relationship("RosbagTopic", back_populates="rosbag")
