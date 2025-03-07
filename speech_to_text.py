import threading
import queue
import alsaaudio
import numpy as np
import whisper
import wave

# Config
DEVICE = "plughw:CARD=sofhdadsp,DEV=6"
CHANNELS = 1
RATE = 16000
PERIOD_SIZE = 1024
CHUNK_DURATION_SEC = 3
# FRAMES_PER_CHUNK = int((RATE / PERIOD_SIZE) * CHUNK_DURATION_SEC)
CHUNK_BYTES = RATE * CHANNELS * 2 * CHUNK_DURATION_SEC
MODEL = "base.en"  # Other models: "tiny", "base", "small", "medium", "large"

# Thread-safe queue
audio_queue = queue.Queue()


def audio_recorder(audio_queue):
    """
    Record the audio.
    """

    inp = alsaaudio.PCM(
        type=alsaaudio.PCM_CAPTURE,
        mode=alsaaudio.PCM_NONBLOCK,
        device=DEVICE,
        channels=CHANNELS,
        rate=RATE,
        format=alsaaudio.PCM_FORMAT_S16_LE,
        periodsize=PERIOD_SIZE
    )

    frames = []
    current_bytes = 0

    print("Recording thread started...")
    while True:
        length, data = inp.read()
        if length:
            frames.append(data)
            current_bytes += len(data)

        if current_bytes >= CHUNK_BYTES:
            audio_buffer = b''.join(frames)
            audio_queue.put(audio_buffer)
            frames.clear()
            current_bytes = 0


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
            with wave.open(f"temp/{text}.wav", "wb") as wf:
                wf.setnchannels(CHANNELS)
                wf.setsampwidth(2)  # 2 bytes per sample (16 bits)
                wf.setframerate(RATE)
                wf.writeframes(audio_buffer)

            audio_queue.task_done()

# Start threads
recorder_thread = threading.Thread(target=audio_recorder, args=(audio_queue,), daemon=True)
transcriber_thread = threading.Thread(target=audio_transcriber, args=(audio_queue,), daemon=True)
recorder_thread.start()
transcriber_thread.start()

# Keep main thread alive
recorder_thread.join()
transcriber_thread.join()
