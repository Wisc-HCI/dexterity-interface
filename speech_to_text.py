import threading
import queue
import alsaaudio
import numpy as np
import whisper
import wave
import audioop

# Config
DEVICE = "plughw:CARD=sofhdadsp,DEV=6"
CHANNELS = 1
RATE = 16000
PERIOD_SIZE = 1024
CHUNK_DURATION= 3
# FRAMES_PER_CHUNK = int((RATE / PERIOD_SIZE) * CHUNK_DURATION_SEC)
PAUSE_DURATION = 0.3  # 300 ms
PAUSE_BYTES = RATE * CHANNELS * 2 * PAUSE_DURATION
CHUNK_BYTES = RATE * CHANNELS * 2 * CHUNK_DURATION
MODEL = "base.en"  # Other models: "tiny", "base", "small", "medium", "large"


# Thread-safe queue
audio_queue = queue.Queue()


def audio_recorder(audio_queue):
    """
    Record the audio.
    """

    inp = alsaaudio.PCM(
        type=alsaaudio.PCM_CAPTURE,
        mode=alsaaudio.PCM_NORMAL,
        device=DEVICE,
        channels=CHANNELS,
        rate=RATE,
        format=alsaaudio.PCM_FORMAT_S16_LE,
        periodsize=PERIOD_SIZE
    )

    # Benchmark background noise before starting with the average noise level
    benchmark_frames = 100
    silence_threshold = 0  
    for _ in range(benchmark_frames):
        length, data = inp.read()
        if length:
            rms = audioop.rms(data, 2)
            silence_threshold += audioop.rms(data, 2)
    silence_threshold /= benchmark_frames
    print(f"SILENCE THRESHOLD: {silence_threshold} RMS")

    frames = []
    current_speaking_bytes = 0
    current_silence_bytes = 0


    print("Recording now...")
    while True:
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
            audio_queue.put(audio_buffer)

            frames.clear()
            current_speaking_bytes = 0
            current_silence_bytes = 0

def audio_transcriber(audio_queue):
    """
    Transcribes the audio with whisper.
    """
    audio_model = whisper.load_model(MODEL)
    print("Transcription thread started...")

    while True:
        if not audio_queue.empty():
            audio_buffer = audio_queue.get()

            # Convert data from 16 bit wide integers to floating point with a width of 32 bits.
            # Clamp the audio stream frequency to a PCM wavelength compatible default of 32768hz max.
            audio_np = np.frombuffer(audio_buffer, dtype=np.int16).astype(np.float32) / 32768.0

            result = audio_model.transcribe(audio_np, language='en')
            text = result['text'].strip()

            print(text)
            # with wave.open(f"temp/{text}.wav", "wb") as wf:
            #     wf.setnchannels(CHANNELS)
            #     wf.setsampwidth(2)  # 2 bytes per sample (16 bits)
            #     wf.setframerate(RATE)
            #     wf.writeframes(audio_buffer)

            audio_queue.task_done()

# Start threads
recorder_thread = threading.Thread(target=audio_recorder, args=(audio_queue,), daemon=True)
transcriber_thread = threading.Thread(target=audio_transcriber, args=(audio_queue,), daemon=True)
recorder_thread.start()
transcriber_thread.start()

# Keep main thread alive
recorder_thread.join()
transcriber_thread.join()
