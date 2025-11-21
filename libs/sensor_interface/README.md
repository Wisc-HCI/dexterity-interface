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
```


## Running k4a viewer:
```bash
# Linux
sudo k4aviewer
```