# Dexterity Interface



## Prerequisites
You will need [Docker Engine](https://docs.docker.com/engine/install/).

## Setup
1. Setup this repo.

    Build the container image and start the container. Make sure you are in this root directory. These commands mount on the current directory as the containers file system so any changes you make to the files on your host machine will be mirrored in the container. The container is in privelaged mode to have access to your mic.
    ```bash
    sudo docker build -t dex-interface .

    sudo docker run --rm -it --privileged -v $(pwd):/workspace --net=host dex-interface
    ```


## Running


* To run the llm script, run:
```bash
python3 chat.py
```

* To run the speech-to-text , run:
```bash
pulseaudio --system --disallow-exit --disallow-module-loading --verbose --daemonize

whisper test.m4a  --model tiny --language English

```

```bash
pulseaudio --check
```


## Resources
https://github.com/openai/whisper   