import alsaaudio
import wave

# Configuration

DEVICE = "plughw:CARD=sofhdadsp,DEV=6"
CHANNELS = 1
RATE = 44100
PERIOD_SIZE = 1024
# DURATION = 5  # seconds
OUTPUT_FILE = "test.wav"


inp = alsaaudio.PCM(type=alsaaudio.PCM_CAPTURE, mode=alsaaudio.PCM_NONBLOCK,   
                     device=DEVICE,
                    channels=CHANNELS,  periodsize=PERIOD_SIZE)


# Prepare WAV file
wf = wave.open(OUTPUT_FILE, 'wb')
wf.setnchannels(CHANNELS)
wf.setsampwidth(2)  # 16-bit = 2 bytes per sample
wf.setframerate(RATE)

frames = []
num_loops = 1000000

print(f"Recording from {DEVICE}")

for _ in range(num_loops):
    length, data = inp.read()
    if length:
        frames.append(data)

# Save to file
wf.writeframes(b''.join(frames))
wf.close()
print(f"Recording saved to {OUTPUT_FILE}")
