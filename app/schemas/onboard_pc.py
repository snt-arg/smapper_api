from pydantic import BaseModel


class OnboardPCSchema(BaseModel):
    model: str
