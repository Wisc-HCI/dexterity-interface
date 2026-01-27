import os
import json
from planning.llm.llm import LLM
from typing import Optional, List, Dict
from dotenv import load_dotenv

from anthropic import Anthropic

class Claude(LLM):
    def __init__(self, role_description:str, model: Optional[str] = None, save_history:bool = True):
        """
        Initialize the LLM  with a system role description. REQUIRES ANTHROPIC_API_KEY=your-api-key-here in env.

        Args:
            role_description (str): A description of the assistant's role or behavior.
            model (Optional[str]): Optional model override. If not provided, uses
                CLAUDE_MODEL from the environment or defaults to 'claude-sonnet-4-5'.
            save_history (bool): True if each prompt to LLM should include the prior
                chat history
        """
        
        super().__init__(role_description, save_history)
        load_dotenv(override=False)

        api_key = os.getenv("ANTHROPIC_API_KEY", "")
        if not api_key:
            raise RuntimeError(
                "ANTHROPIC_API_KEY=your-api-key-here not found in environment (.env)."
            )

        env_model = os.getenv("CLAUDE_MODEL", "").strip()
        self._model = model or env_model or "claude-sonnet-4-5"


        self._client = Anthropic()  # Automatically uses ANTHROPIC_API_KEY
    

    def prompt(self, input:str) -> str:
        """
        Prompt the LLM, and include prior prompting history and context
        
        Args:
            input (str): The user prompt input
        
        Returns:
            str: Assistant reply content.
        """
        
        messages: List[Dict] = self.history(include_fewshot=True)
        user_msg = {"role": "user", "content": input}
        messages = messages + [user_msg]

        try:
            resp = self._.messages.create(
                model=self._model,
                messages=messages
            )

            reply = resp.content[0].text

            if self._save_history:
                self.chat_history.append(user_msg)
                self.chat_history.append({
                    "role": "assistant",
                    "content": reply,
                })

        except Exception:
            reply = self.prompt(input)
            reply = self._extract_json(reply)

        try:
            json.loads(reply)
        except Exception:
            reply = json.dumps({"raw": reply})

        return reply


    def prompt_json(self, input:str) -> str:
        """
        Request a JSON object response using structured output;
        fall back to plain completion + JSON extraction.

        Args:
            input (str): User prompt input.

        Returns:
            str: JSON string.
        """
        # TODO
        return self.prompt(input)


if __name__ == "__main__":
    claude = Claude("You are helpful task planner.")
    print(claude.prompt("Say hello in one sentence."))