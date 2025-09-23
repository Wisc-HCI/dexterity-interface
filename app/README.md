# Dexterity Interface Application

TODO: figure out if want to use this or ROS web bridge (may be better for recording data, handling streams, etc.)

## Setup

```bash
pip3 install -e app/ui_backend
```
## Running
Make sure you are in root folder (`dexterity-interface`) and have sourced `venv-dex` before running these in seperate terminals:

Start the Isaacsim UI in streaming mode:
```bash
LIVESTREAM=2  python3 -m robot_motion_interface.isaacsim.isaacsim_interface --kit_args="--/app/window/hideUi=true --/app/window/drawMouse=false"
```

Start API server:
```bash
uvicorn ui_backend.api:app --reload
```

Start frontend server:
```bash
npm run dev --prefix app/ui_frontend/
```

NOTE: If you instead want to build and run the frontend for production, run the following:
```bash
npm run build --prefix app/ui_frontend/
npx serve app/ui_frontend/dist
```

In your web browser, go to the following to test:
* API: http://127.0.0.1:8000/api/test
* API docs: http://127.0.0.1:8000/docs
* Front end: http://127.0.0.1:3000
