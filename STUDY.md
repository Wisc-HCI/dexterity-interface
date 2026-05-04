# Study Setup



## ON DESKTOP (4 terminals):
This will launch four terminals:
```bash
sudo apt install tmux ffmpeg
cd setup_scripts
./start_desktop.sh
```

Wait until all commands have finished running in all four terminals.

Then, in terminal 1, run:
```bash
ros2 launch primitives_ros sim.launch.py isaac_args:='--kit_args=--/app/window/hideUi=true'
```


Wait until Terminal 1 says `Creating window for environment.`. 
Then these in Terminal 2  (replace the PID and TASK that of your participant):
```bash
export PID=REPLACE TASK=REPLACE # EXAMPLE: export PID=1 TASK=1

uvicorn ui_backend.api:app
```

Run this in terminal 3 to start the frontend:
```bash
npm run dev --prefix app/ui_frontend/
```

Navigate to one of these in the browser (make sure there is ONLY 1 interface tab open or the sim won't show up):
```bash
# CONDITION 1 (no plan)
http://127.0.0.1:3000?show_plan=false&logging=true

# CONDITION 2 (no plan editing/interaction)
http://127.0.0.1:3000?plan_interaction=false&logging=true

# CONDITION 3 (full system)
http://127.0.0.1:3000?logging=true
```

You may need to refresh a couple of times before the sim appears.


When your ready, run this in terminal 4 to start recording (replace the PID with that of your participant):
```bash
export PID=REPLACE  # EXAMPLE: export PID=1

sudo ffmpeg -framerate 30 -f x11grab -i $DISPLAY app/experiment_logging/PID_$PID/recording_$(date +%Y-%m-%d_%H-%M-%S).mp4
```

## ON LAPTOP (single terminal):

```bash
cd setup_scripts
./start_laptop.sh
```

Then once all the scripts in the terminal finish, run the following:
```bash
ros2 launch primitives_ros real.launch.py
```


## If camera needs calibration (run on Desktop in terminal 1)
```bash
cd libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config
python calibrate_T_world_color.py --config realsense_config.yaml --marker-size 0.1 --marker-pos 0 0 0.9369 --write
```