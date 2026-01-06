from pydantic import BaseModel
from typing import Optional, Literal, List

class Task(BaseModel):
    """
    User inputted task prompt.
    """
    task: str

class Primitive(BaseModel):
    """
    Individual Primitive
    """
    name: str
    parameters: dict = None
    core_primitives: Optional[List["Primitive"]] = None

class Execution(BaseModel):
    """
    Message to be returned after execution of
    primitives complete
    """
    success: bool
    executed_on: Literal["real", "sim"]