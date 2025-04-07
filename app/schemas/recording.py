from typing import List, Optional
from pydantic import BaseModel


class RecordingMetadata(BaseModel):
    bag_name: str
    start_time: float
    elapsed_time: float
    stop_time: Optional[float] = None
    bag_size: int
    topics: List[str]


class RecordingStatus(BaseModel):
    state: str
    metadata: Optional[RecordingMetadata] = None
