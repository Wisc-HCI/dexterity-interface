from planning.llm.llm import LLM
from abc import abstractmethod

class GPT(LLM):
    def __init__(self, role_description):
        """
        Initialize the LLM  with a system role description.

        Args:
            role_description (str): A description of the assistant's role or behavior.
        """
        super().__init__(role_description)
    

    def prompt(self, input:str) -> str:
        """
        Prompt the LLM, and include prior prompting history and context
        
        Args:
            input (str): The user prompt input
        """
        ...


if __name__ == "__main__":
    gpt = GPT("You are helpful task planner.")