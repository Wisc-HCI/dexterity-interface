import alsaaudio
import wave
import whisper
import numpy as np

# Configuration

DEVICE = "plughw:CARD=sofhdadsp,DEV=6"
CHANNELS = 1
RATE = 16000
PERIOD_SIZE = 1024

MODEL = "base.en"  # Whisper model: ["tiny", "base", "small", "medium", "large"]


inp = alsaaudio.PCM(type=alsaaudio.PCM_CAPTURE, mode=alsaaudio.PCM_NONBLOCK,   
                    device=DEVICE, rate=RATE,
                    channels=CHANNELS,  periodsize=PERIOD_SIZE)


# Load Whisper model
audio_model = whisper.load_model(MODEL)

frames = []
num_loops = 5000000

print(f"Recording from {DEVICE}")

for _ in range(num_loops):
    length, data = inp.read()
    if length:
        frames.append(data)

print("Recording Ended.")
# Concatenate all frame buffers into one single bytes object
audio_buffer = b''.join(frames)

with wave.open("test_audio.wav", "wb") as wf:
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(2)  # 2 bytes per sample (16 bits)
    wf.setframerate(RATE)
    wf.writeframes(audio_buffer)

# Convert in-ram buffer to something the model can use directly without needing a temp file.
# Convert data from 16 bit wide integers to floating point with a width of 32 bits.
# Clamp the audio stream frequency to a PCM wavelength compatible default of 32768hz max.
audio_np = np.frombuffer(audio_buffer, dtype=np.int16).astype(np.float32) / 32768.0

result = audio_model.transcribe(audio_np)
text = result['text'].strip()

print("TEXT!:")
print(text)