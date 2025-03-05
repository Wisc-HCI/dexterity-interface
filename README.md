# Dexterity Interface



## Prerequisites
* You will need a Ubuntu machine (sorry, but getting audio working in containers is very challenging)
* You will need [Docker Engine](https://docs.docker.com/engine/install/).

## Setup
1. Setup this repo.

    Build the container image and start the container. Make sure you are in this root directory. These commands mount on the current directory as the containers file system so any changes you make to the files on your host machine will be mirrored in the container. The container is in privileged mode to have access to your mic.
    ```bash
    sudo docker build -t dex-interface .

    sudo docker run --rm -it --privileged --device /dev/snd --group-add audio -v $(pwd):/workspace --net=host dex-interface
    ```

2. Setup audio in the container (sorry this is a bit complicated)

    a. 

## Running


* To run the llm script, run:
```bash
python3 chat.py
```

* To run the speech-to-text , run:
```bash
pulseaudio --start

whisper test.m4a  --model tiny --language English

```

```bash
pulseaudio --check
pactl list short sinks

aplay -l # Output devices



arecord -l # Input devices devices

export ALSA_CARD=sofhdadsp
export ALSA_CAPTURE_DEVICE=hw:1,6

speaker-test -t wav -c 6  # Speaker test with voice

arecord -D plughw:1,6 --format=cd file.wav  # For hw:x,y -> x is card, y is device



# Works locally
arecord --format=cd file.wav

```


```
card 1: sofhdadsp [sof-hda-dsp], device 6: DMIC (*) []
  Subdevices: 1/1
  Subdevice #0: subdevice #0
  ```


## Resources
https://github.com/openai/whisper   

https://github.com/mviereck/x11docker/wiki/Container-sound:-ALSA-or-Pulseaudio