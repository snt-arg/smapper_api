from pydantic import BaseModel


class OnboardPCSchema(BaseModel):
    """Schema representing metadata about the onboard PC.

    Attributes:
        model: The model name or identifier of the onboard PC.
    """

    model: str
