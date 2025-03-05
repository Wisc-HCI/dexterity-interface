import alsaaudio
import wave
import whisper

# Configuration

DEVICE = "plughw:CARD=sofhdadsp,DEV=6"
CHANNELS = 1
RATE = 16000
PERIOD_SIZE = 1024

MODEL = "base.en"  # Whisper model: ["tiny", "base", "small", "medium", "large"]


inp = alsaaudio.PCM(type=alsaaudio.PCM_CAPTURE, mode=alsaaudio.PCM_NONBLOCK,   
                     device=DEVICE,
                    channels=CHANNELS,  periodsize=PERIOD_SIZE)


# Load Whisper model
audio_model = whisper.load_model(MODEL)

frames = []
num_loops = 1000000

print(f"Recording from {DEVICE}")

for _ in range(num_loops):
    length, data = inp.read()
    if length:
        frames.append(data)
        
result = audio_model.transcribe(frames)
text = result['text'].strip()