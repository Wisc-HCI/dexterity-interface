# Dexterity Planning

This module provides LLM-based task planning for the dexterity interface.  
It uses a GPT model to convert natural language tasks into structured JSON primitive plans,  
based on a catalog of low- and mid-level manipulation primitives.

---

## Setup

```bash
# Create and activate virtual environment
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate

# Install planning package and dependencies
pip install -e libs/planning/planning_py

# Add a .env file with the following format
# (place it in libs/planning)
# Example:
OPENAI_API_KEY=your_openai_api_key_here
OPENAI_MODEL=gpt-5-nano

# Run the example to test the planner
python -m planning.examples.primitive_breakdown