from abc import ABC, abstractmethod
from typing import List, Dict

class LLM(ABC):
    def __init__(self, role_description:str):
        """
        Initialize the LLM  with a system role description.

        Args:
            role_description (str): A description of the assistant's role or behavior.
        """
        
        self.chat_history: List[Dict] = [{
            "role": "developer", 
            "content": role_description,
        }]
    

    def add_fewshot(self, example_query:str, example_response:str):
        """
        Add a few-shot example to the chat history.

        Args:
            example_query (str): Example input or question from the user.
            example_response (str): Example response from the assistant.
        """

        self.chat_history += [
            {"role": "user", "content": example_query, "fewshot": True},
            {"role": "assistant", "content": example_response, "fewshot": True},
        ]


    def history(self, include_fewshot:bool = False) -> list[dict]:
        """
        Return the chat history.

        Args:
            include_fewshot (bool): Whether to include few-shot examples in
                the returned history. Defaults to False.

        Returns:
            list[dict]: The conversation history in the format:
                [{"role": "", "content": ""}, ...]
        """

        if include_fewshot:
            return list(self.chat_history)

        filtered: List[Dict] = []
        for msg in self.chat_history:
            if msg.get("fewshot", False):
                continue
            filtered.append(msg)
        return filtered

    
    def reset(self):
        """ 
        Clear all chat history, except role description
        """

        if self.chat_history:
            self.chat_history = [self.chat_history[0]]
        else:
            self.chat_history = []


    @abstractmethod
    def prompt(self, input:str) -> str:
        """
        Prompt the LLM, and include prior prompting history and context
        
        Args:
            input (str): The user prompt input
        """
        ...


    def prompt_json(self, input:str) -> str:
        """
        Ask the LLM for a JSON-only response. Implemented in child classes
        that support structured output; falls back to plain prompt().

        Args:
            input (str): The user prompt input.

        Returns:
            str: JSON string (ideally a single JSON object).
        """
        
        return self.prompt(input)
