# Dexterity Interface



## Prerequisites
* You will need a Ubuntu machine (sorry, but getting audio working in containers is very challenging)
* You will need [Docker Engine](https://docs.docker.com/engine/install/).
* [Optional] If have a Nvidia GPU, and you would like to use it with this docker container, [install](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-the-nvidia-container-toolkit) and [configure](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuration) the Nvidia Container Toolkit.

## Setup
1. Setup this repo.

    a. Build the container image and start the container. 
        ```bash
    sudo docker build -t dex-interface .
    ```
    
    b Make sure you are in this root directory. These commands mount on the current directory as the containers file system so any changes you make to the files on your host machine will be mirrored in the container. The container is in privileged mode to have access to your mic. Note: privileged mode is alright as a local devolvement environment, but don't treat it as it's own isolated, secure system.
    ```bash
    # Either run without Nvidia GPU
    sudo docker run --rm -it --privileged  -v $(pwd):/workspace --net=host dex-interface

    # OR WITH Nvidia GPU
    sudo docker run --rm -it --privileged --runtime=nvidia --gpus all  -v $(pwd):/workspace --net=host dex-interface
    ```

2. Setup audio in the container (sorry this is a bit complicated)

    a. Run `arecord -l` to see your output devices (microphones). You should see a list of devices like this:
    ```
    card 1: sofhdadsp [sof-hda-dsp], device 6: DMIC (*) []
    Subdevices: 1/1
    Subdevice #0: subdevice #0
    ```
    Notice the Card name (C) and Device number (D) number. In this example, C is sofhdadsp and D is 6.

    b. Export the following with YOUR C and D numbers `export CARD="C" DEVICE="D"`. In our example, we would run `export CARD="sofhdadsp" DEVICE="6"`.



## Running

* To run the llm script, run:
    ```bash
    python3 chat.py
    ```

* To run the speech-to-text live, run:
    ```bash
    python3 Transcribe.py # Requires env set in setup 2b)
    ```

* To run the speech-to-text from recording, run:
    ```bash
    whisper test.wav --model tiny --language English --output_dir ./temp
    ```


## Testing
Run the following in the container to help troubleshoot.
```bash
# Troubleshooting Mic/Audio
aplay -l  # Shows output audio devices
arecord -l  # Shows input audio devices

speaker-test -t wav -c 6  # Speaker test with voice
arecord -D plughw:$CARD,$DEVICE --format=cd test.wav   # Recording test (requires env set in setup 2b)

# Troubleshooting Nvidia GPU (if setup)
python3 -c "import torch; print(torch.cuda.is_available())" # True if can see GPU, False if not

```





## Resources
https://github.com/openai/whisper   

https://github.com/mviereck/x11docker/wiki/Container-sound:-ALSA-or-Pulseaudio

https://github.com/larsimmisch/pyalsaaudio/blob/main/recordtest.py

https://github.com/davabase/whisper_real_time/blob/master/transcribe_demo.py
