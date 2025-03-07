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

    a. Run `arecord -l` to see your output devices (microphones). You should see a list of devices like this:
    ```
    card 1: sofhdadsp [sof-hda-dsp], device 6: DMIC (*) []
    Subdevices: 1/1
    Subdevice #0: subdevice #0
    ```
    Notice the Card (C) and Device (D) number. In this example, C is 1 and D is 6.

    b. Export the following with YOUR C and D numbers `export ALSA_MIC=C,D`. In our example, we would run `export ALSA_MIC=1,6`.



## Running


* To run the llm script, run:
```bash
python3 chat.py
```

* To run the speech-to-text , run:
```bash
whisper test.wav --model tiny --language English --output_dir ./temp
```


## Testing

```bash
aplay -l  # Shows output audio devices
arecord -l  # Shows input audio devices

speaker-test -t wav -c 6  # Speaker test with voice
arecord -D plughw:$ALSA_MIC --format=cd test.wav   # Recording test (requires env set in setup 2b)


```





## Resources
https://github.com/openai/whisper   

https://github.com/mviereck/x11docker/wiki/Container-sound:-ALSA-or-Pulseaudio

https://github.com/larsimmisch/pyalsaaudio/blob/main/recordtest.py

https://github.com/davabase/whisper_real_time/blob/master/transcribe_demo.py
