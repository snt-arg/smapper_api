from typing import Optional
from pydantic import BaseModel


class TopicSchema(BaseModel):
    name: str
    msg_type: str
    message_count: Optional[int]
