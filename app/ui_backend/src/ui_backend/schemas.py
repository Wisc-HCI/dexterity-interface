from pydantic import BaseModel
from typing import Optional, Literal

class Task(BaseModel):
    """
    User inputted task prompt.
    """
    task: str

class Primitive(BaseModel):
    """
    Individual Primitive
    """
    type: str
    arm: Optional[str] = None
    pose: Optional[list[float]] = None

class Execution(BaseModel):
    """
    Message to be returned after execution of
    primitives complete
    """
    success: bool
    executed_on: Literal["real", "sim"]