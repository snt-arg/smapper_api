from pydantic import BaseModel


class OnboardPC(BaseModel):
    model: str
