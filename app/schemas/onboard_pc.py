from pydantic import BaseModel, Field


class OnboardPCSchema(BaseModel):
    """Schema representing metadata about the onboard PC."""

    model: str = Field(description="The model name or identifier of the onboard PC.")
