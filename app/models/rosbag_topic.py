from sqlalchemy import Column, Integer, String, ForeignKey
from sqlalchemy.orm import relationship
from app.db.database import Base


class RosbagTopic(Base):
    __tablename__ = "rosbag_topics"

    id = Column(Integer, primary_key=True, index=True)
    rosbag_id = Column(Integer, ForeignKey("rosbag_metadata.id"))
    name = Column(String, nullable=False)
    msg_type = Column(String, nullable=False)

    rosbag = relationship("RosbagMetadata", back_populates="topics")
