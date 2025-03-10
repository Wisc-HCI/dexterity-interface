"""
This script transcribes audio with whisper. The first time it is run,
it will take a couple seconds to download the model.
"""

import threading
import os
import queue
import alsaaudio
import numpy as np
import whisper
import wave
import audioop
import functools

class Transcribe:
    def __init__(self, model:str ='base.en'):
        """
        model: Whisper model. Options are "base.en" [default], "tiny.en", "small.en", "medium.en", "large.en", "turbo".
                See details here: https://github.com/openai/whisper?tab=readme-ov-file#available-models-and-languages
        """
        # Config
        self.device = "plughw:CARD=sofhdadsp,DEV=6"

        # Load model
        print(f"Loading {model} model...")
        whisper.torch.load = functools.partial(whisper.torch.load, weights_only=True) # Prevents pytorch FutureWarning
        self.audio_model = whisper.load_model(model)

        # Thread-safe queue
        self.audio_queue = queue.Queue()

        # Transcription String
        self.transcription = ""

        # Stops gracefully
        self.stop_event = threading.Event()
    
    def start_transcribing(self):
        """
        Triggers transcription.
        """

        # Start threads
        recorder_thread = threading.Thread(target=self._audio_recorder, args=(), daemon=True)
        transcriber_thread = threading.Thread(target=self._audio_transcriber, args=(), daemon=True)
        recorder_thread.start()
        transcriber_thread.start()

    
    def stop_transcribing(self):
        """
        Stops the transcription process gracefully.
        """
        
        self.stop_event.set()
        print("\nStopped transcription.")


    def get_current_transcription(self) -> str:
        """
        Returns current transcription.
        """
        
        return self.transcription


    def _audio_recorder(self):
        """
        Record the audio.
        """

        RATE = 16000
        CHANNELS = 1
        PERIOD_SIZE = 1024
        CHUNK_DURATION= 3
        # FRAMES_PER_CHUNK = int((RATE / PERIOD_SIZE) * CHUNK_DURATION_SEC)
        PAUSE_DURATION = 0.3  # 300 ms
        PAUSE_BYTES = RATE * CHANNELS * 2 * PAUSE_DURATION
        CHUNK_BYTES = RATE * CHANNELS * 2 * CHUNK_DURATION
        BENCHMARK_FRAMES = 100

        inp = alsaaudio.PCM(
            type=alsaaudio.PCM_CAPTURE,
            mode=alsaaudio.PCM_NORMAL,
            device=self.device,
            channels=CHANNELS,
            rate=RATE,
            format=alsaaudio.PCM_FORMAT_S16_LE,
            periodsize=PERIOD_SIZE
        )

        # Benchmark background noise before starting with the average noise level
        silence_threshold = 0  
        for _ in range(BENCHMARK_FRAMES):
            length, data = inp.read()
            if length:
                rms = audioop.rms(data, 2)
                silence_threshold += audioop.rms(data, 2)
        silence_threshold /= BENCHMARK_FRAMES
       

        frames = []
        current_speaking_bytes = 0
        current_silence_bytes = 0


        print("Recording now...")
        while not self.stop_event.is_set():
            length, data = inp.read()
            if length:

                # Check silence Threshold
                rms = audioop.rms(data, 2)
                frames.append(data)
                current_speaking_bytes += len(data)

                if rms >= silence_threshold: # Not silent 
                    current_silence_bytes = 0
                else:
                    current_silence_bytes += len(data)

            # Add to audio to queue if chunk is too long or there is a pause
            if current_speaking_bytes >= CHUNK_BYTES or current_silence_bytes >= PAUSE_BYTES:

                audio_buffer = b''.join(frames)
                self.audio_queue.put(audio_buffer)

                frames.clear()
                current_speaking_bytes = 0
                current_silence_bytes = 0

    def _audio_transcriber(self):
        """
        Transcribes the audio with whisper.
        """

        while not self.stop_event.is_set():
            if  self.audio_queue.empty():
                time.sleep(0.1) # Gives time for other threads to run
            else:
                audio_buffer = self.audio_queue.get()  

                # Convert data from 16 bit wide integers to floating point with a width of 32 bits.
                # Clamp the audio stream frequency to a PCM wavelength compatible default of 32768hz max.
                audio_np = np.frombuffer(audio_buffer, dtype=np.int16).astype(np.float32) / 32768.0

                result = self.audio_model.transcribe(audio_np, fp16=False)
                text = result['text'].strip()
                self.transcription += ' ' + text

                self.audio_queue.task_done()



# Example for using this class by just printing output
if __name__ == "__main__":
    import time

    transcribe = Transcribe(model="base.en")
    transcribe.start_transcribing()
    last_transcription = ""
    try:
        while True:
            current_text = transcribe.get_current_transcription()
            if current_text != last_transcription:
                os.system('clear')
                print(current_text)
                last_transcription = current_text
            time.sleep(0.1) # Gives time for other threads to run
    except KeyboardInterrupt:
        transcribe.stop_transcribing()
