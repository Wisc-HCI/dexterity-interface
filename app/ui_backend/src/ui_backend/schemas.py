from pydantic import BaseModel
from typing import Optional, Literal, List


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

class Plan(BaseModel):
    id: str
    revision_of: Optional[str] = None
    task_prompt: str
    primitive_plan: List[Primitive]


class NewPlan(BaseModel):
    revision_of: Optional[str] = None
    task_prompt: str


class RevisedPlan(BaseModel):
    revision_of: Optional[str] = None
    task_prompt: str
    primitive_plan: List[Primitive]