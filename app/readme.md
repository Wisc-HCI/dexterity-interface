# Dexterity Interface Application

TODO: figure out if want to use this or ROS web bridge (may be better for recording data, handling streams, etc.)

## Setup

```bash
pip3 install -e app/ui_backend
```
## Running
Make sure you are in root folder (`dexterity-interface`) before running these in seperate terminals:

Start API server:
```bash
uvicorn ui_backend.api:app --reload
```

Start frontend server:
```bash
python -m http.server 5500 --directory app/ui_frontend
```

In your web browser, go to the following to test:
* API: http://127.0.0.1:8000/api/test
* API docs: http://127.0.0.1:8000/docs
* Front end: http://127.0.0.1:5500
