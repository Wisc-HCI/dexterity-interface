# Dexterity Interface

## Setup
1. First, you need to create a .env file in this folder with the OpenAI credentials. It should be in this format:
    ```bash
    OPENAI_API=YOUR_API_KEY_HERE
    ```

2. Next, start a python virtual environment in this directory:

    Linux/Mac:
    ```bash
    python3 -m venv venv
    source venv/bin/activate  
    ```

    Windows:
    ```powershell
    python3 -m venv venv
    .\venv\Scripts\Activate.ps1
    ```

    If this windows command doesn't work, you may have to run this in an Admin shell first:
    ```powershell
    set-executionpolicy remotesigned
    ```

3. Next, install all the python requirements:
    ```bash
    pip install -r requirements.txt
    ```


## Running
* To run the llm script, run:
```bash
python3 chat.py
```
Note: if your venv has become deactivated, you may need to reactivate it with the activate command in the Setup section.


## Resources:
* https://github.com/justagist/mujoco_panda