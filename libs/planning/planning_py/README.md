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
# (place it in the project root or libs/planning/planning_py/)
# Example:
OPENAI_API_KEY=your_openai_api_key_here
OPENAI_MODEL=gpt-5-nano

# Run the example to test the planner
python -m planning.examples.primitive_breakdown
```

---

## Test MOVE_TRANSPORT Preconditions

This task verifies that the planner correctly loads and respects preconditions for the
`MOVE_TRANSPORT` primitive using a YAML configuration file.  
It generates a structured JSON output for **3 scenes × 4 precondition sets = 12 cases**.

### Run

```bash
# Run from project root
python -m planning.examples.test_preconditions
