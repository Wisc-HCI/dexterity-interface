import os
import re
import base64
import io
from PIL import Image
import numpy as np
import json
from planning.vlm.vlm import VLM
from typing import Optional, List, Dict
from dotenv import load_dotenv

try:
    from openai import OpenAI
except Exception:
    OpenAI = None


def numpy_to_base64(img: np.ndarray) -> str:
    """
    Helper that transforms numpy format image to a base64 object amenable for VLM prompting.

    Args:
        img (np.ndarray): Image of the failure scene. Can be (H, W, 3) float RGB [0,1] or [0,255].

    Returns:
        img (str): base64 string
    """
    if img.dtype != np.uint8:
        img = np.clip(img * 255, 0, 255).astype(np.uint8)

    pil_img = Image.fromarray(img, mode="RGB")
    buf = io.BytesIO()
    pil_img.save(buf, format="PNG")
    return base64.b64encode(buf.getvalue()).decode("utf-8")


class GPT(VLM):
    def __init__(self, role_description:str, model: Optional[str] = None, save_history:bool = True):
        """
        Initialize the VLM  with a system role description.

        Args:
            role_description (str): A description of the assistant's role or behavior.
            model (Optional[str]): Optional model override. If not provided, uses
                OPENAI_VLM_MODEL from the environment or defaults to 'gpt-5-nano'.
            save_history (bool): True if each prompt to VLM should include the prior
                chat history
        """
        
        super().__init__(role_description, save_history)
        load_dotenv(override=False)

        api_key = os.getenv("OPENAI_API_KEY", "")
        if not api_key:
            raise RuntimeError(
                "OPENAI_API_KEY not found in environment (.env)."
            )

        env_model = os.getenv("OPENAI_VLM_MODEL", "").strip()
        self._model = model or env_model or "gpt-4.1-mini"
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
        Prompt the VLM, and include prior prompting history and context
        
        Args:
            input (str): The user prompt input
        
        Returns:
            str: Assistant reply content.
        """

        messages: List[Dict] = self.history(include_fewshot=True)

        user_msg = {"role": "user", "content": input}

        messages = messages + [user_msg]


        params = dict(model=self._model, messages=messages)

        resp = self._client.chat.completions.create(**params)
        reply = resp.choices[0].message.content or ""

        if self._save_history:
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

            resp = self._client.chat.completions.create(**params)

            reply = resp.choices[0].message.content or "{}"

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

    def _prompt_with_image(self, prompt_text: str, image_np: np.ndarray) -> str:
        """
        Build strict JSON instruction template by combining text and image.

        Args:
            prompt_text (str): Text instruction, output from _prompt_template.
            image_np (np.ndarray): Image of the failure scene.

        Returns:
            str: Complete prompt string.
        """

        image_b64 = numpy_to_base64(image_np)
        input_content = [
        {"type": "input_text", "text": prompt_text},
        {
            "type": "input_image",
            "image_url": f"data:image/png;base64,{image_b64}"
        }
        ]

        resp = self._client.responses.create(
            model="gpt-4.1-mini",
            input=[
                {
                    "role": "user",
                    "content": input_content
                }
            ],
            #response_format={"type": "json_object"}, # not supported in many mini models :(
        )

        reply = resp.output_text or "{}"
        return reply
    
if __name__ == "__main__":
    gpt = GPT("You are helpful task planner and problem solver.")
    print(gpt.prompt("Say hello in one sentence."))