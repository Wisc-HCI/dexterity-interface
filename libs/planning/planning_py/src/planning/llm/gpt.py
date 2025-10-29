import os
import re
import json
from planning.llm.llm import LLM
from typing import Optional, List, Dict
from dotenv import load_dotenv

try:
    from openai import OpenAI
except Exception:
    OpenAI = None

class GPT(LLM):
    def __init__(self, role_description:str, model: Optional[str] = None):
        """
        Initialize the LLM  with a system role description.

        Args:
            role_description (str): A description of the assistant's role or behavior.
            model (Optional[str]): Optional model override. If not provided, uses
                OPENAI_MODEL from the environment or defaults to 'gpt-5-nano'.
        """
        
        super().__init__(role_description)
        load_dotenv(override=False)

        api_key = os.getenv("OPENAI_API_KEY", "")
        if not api_key:
            raise RuntimeError(
                "OPENAI_API_KEY not found in environment (.env)."
            )

        env_model = os.getenv("OPENAI_MODEL", "").strip()
        self._model = model or env_model or "gpt-5-nano"

        if OpenAI is None:
            raise RuntimeError("openai SDK not installed. Add `openai>=1.40.0` to dependencies.")

        self._client = OpenAI(api_key=api_key)
    

    def _extract_json(self, text:str) -> str:
        """
        Extract the largest JSON object from free-form text.

        Args:
            text (str): Raw model output possibly containing prose.

        Returns:
            str: JSON string (object) if found, else original text.
        """
        
        matches = list(re.finditer(r"\{.*\}", text, flags=re.DOTALL))
        if not matches:
            return text
        start, end = matches[0].span()
        for m in matches[1:]:
            s, e = m.span()
            if (e - s) > (end - start):
                start, end = s, e
        return text[start:end]
    

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

        params = dict(model=self._model, messages=messages)

        if not self._model.startswith("gpt-5-nano"):
            params["temperature"] = 0.2

        resp = self._client.chat.completions.create(**params)
        reply = resp.choices[0].message.content or ""

        self.chat_history.append(user_msg)
        self.chat_history.append({
            "role": "assistant",
            "content": reply,
        })

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
        
        messages: List[Dict] = self.history(include_fewshot=True)
        user_msg = {"role": "user", "content": input}
        messages = messages + [user_msg]

        try:
            params = dict(
                model=self._model,
                messages=messages,
                response_format={"type": "json_object"},
            )
            if not self._model.startswith("gpt-5-nano"):
                params["temperature"] = 0.0

            resp = self._client.chat.completions.create(**params)

            reply = resp.choices[0].message.content or "{}"
        except Exception:
            reply = self.prompt(input)
            reply = self._extract_json(reply)

        try:
            json.loads(reply)
        except Exception:
            reply = json.dumps({"raw": reply})

        return reply


if __name__ == "__main__":
    gpt = GPT("You are helpful task planner.")
    print(gpt.prompt("Say hello in one sentence."))