# Sensor Interface

These are instructions for installing on Ubuntu 22.04 with Python3.11. Recomend doing this in a venv.
## Kinect Setup Instructions
Install dependencies
```bash
sudo apt install python3.11 python3.11-dev python3.11-venv build-essential cmake
sudo apt install build-essential cmake

wget http://archive.ubuntu.com/ubuntu/pool/universe/libs/libsoundio/libsoundio1_1.1.0-1_amd64.deb
sudo dpkg -i libsoundio1_1.1.0-1_amd64.deb
```


Download dependencies
```bash
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/microsoft.gpg > /dev/null

# Requires old version even if running on 22.04
curl -sSL -O https://packages.microsoft.com/config/ubuntu/18.04/packages-microsoft-prod.deb

sudo dpkg -i packages-microsoft-prod.deb

sudo apt update
sudo apt install libk4a1.4 libk4a1.4-dev k4a-tools
```


Setup path
```bash
echo 'export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```


Install python packages
```bash
pip install pyk4a
pip install -e libs/sensor_interface/sensor_interface_py  # run from repo root
```
The editable install registers `sensor_interface` on your path, so no manual `PYTHONPATH` export is needed.

Set USB permissions (no sudo required to open the device)
```bash
cat <<'EOF' | sudo tee /etc/udev/rules.d/99-k4a.rules
SUBSYSTEMS=="usb", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097a", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097b", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097c", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097d", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097e", MODE:="0666"
KERNEL=="video*", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097c", MODE:="0666"
KERNEL=="video*", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097d", MODE:="0666"
KERNEL=="hidraw*", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097e", MODE:="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
# Unplug/replug the Kinect after applying the rules
```

## Kinect streaming example (Python/OpenCV)
Once dependencies above are installed, you can view the RGB-D stream using the provided interface and YAML config:
```bash
# From repo root; uses the sample calibration at camera/config/kinect_config.yaml
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export QT_QPA_PLATFORM=xcb  # avoid Wayland Qt plugin warning
venv-dex/bin/python libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/examples/kinect_stream.py \
    --config libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/kinect_config.yaml
```
Use `--fps`, `--align color|depth`, `--device`, or `--serial` flags as needed. Exit the window with `q` or `Esc`.
You can also close the OpenCV window via the titlebar close button or Ctrl+C in the terminal.


## Running k4a viewer:
```bash
# Linux
sudo k4aviewer
```
