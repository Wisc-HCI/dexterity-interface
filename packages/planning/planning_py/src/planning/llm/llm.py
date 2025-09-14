from abc import abstractmethod

class LLM:
    def __init__(self, role_description):
        """
        Initialize the LLM  with a system role description.

        Args:
            role_description (str): A description of the assistant's role or behavior.
        """
        self.chat_history = [{
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

        self.chat_history += [{"role": "user", "content": example_query}, {"role": "assistant", "content": example_response}]


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
        # TODO
        ...

    
    def reset(self):
        """ 
        Clear all chat history, except role description
        """
        # TODO
        ...


    @abstractmethod
    def prompt(self, input:str) -> str:
        """
        Prompt the LLM, and include prior prompting history and context
        
        Args:
            input (str): The user prompt input
        """
        ...


