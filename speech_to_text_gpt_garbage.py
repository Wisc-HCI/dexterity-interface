import os
import numpy as np
import alsaaudio
import whisper
import torch

from datetime import datetime, timedelta, timezone
from queue import Queue
from time import sleep

# Configuration
MODEL = "base.en"  # Whisper model: ["tiny", "base", "small", "medium", "large"]
DEVICE = "plughw:CARD=sofhdadsp,DEV=6"  
CHANNELS = 1
RATE = 16000  # Whisper works best at 16kHz
PERIOD_SIZE = 1024
PHRASE_TIMEOUT = 3  # Time after which a new phrase is considered started
RECORD_TIMEOUT = 2  # Maximum length per recording chunk

def main():
    # Load Whisper model
    audio_model = whisper.load_model(MODEL)

    # Setup ALSA audio capture
    inp = alsaaudio.PCM(type=alsaaudio.PCM_CAPTURE, mode=alsaaudio.PCM_NONBLOCK,   
                     device=DEVICE,
                    channels=CHANNELS,  periodsize=PERIOD_SIZE)
    
    print("Model loaded. Listening...")

    data_queue = Queue()
    phrase_time = None
    transcription = [""]

    while True:
        try:
            now = datetime.now(timezone.utc)
            length, data = inp.read()
            
            if length:
                data_queue.put(data)

            # Check if there's enough data to process
            if not data_queue.empty():
                phrase_complete = False
                
                if phrase_time and now - phrase_time > timedelta(seconds=PHRASE_TIMEOUT):
                    phrase_complete = True
                
                phrase_time = now

                # Combine audio data
                audio_data = b''.join(list(data_queue.queue))
                data_queue.queue.clear()

                # Convert audio to numpy array (16-bit PCM -> float32)
                audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

                # Transcribe audio
                result = audio_model.transcribe(audio_np, fp16=torch.cuda.is_available())
                text = result['text'].strip()

                # Append or update transcription
                if phrase_complete:
                    transcription.append(text)
                else:
                    transcription[-1] = text

                # Clear screen and print transcription
                # os.system('clear')
                for line in transcription:
                    print(line)
                # print('', end='', flush=True)

            else:
                sleep(0.1)  # Avoid excessive CPU usage

        except KeyboardInterrupt:
            break

    print("\n\nFinal Transcription:")
    for line in transcription:
        print(line)


if __name__ == "__main__":
    main()
