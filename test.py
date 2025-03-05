import speech_recognition as sr

def test_recording():
    recognizer = sr.Recognizer()
    mic = sr.Microphone(device_index=7)

    with mic as source:
        print("Adjusting for ambient noise... Please wait.")
        recognizer.adjust_for_ambient_noise(source)
        print("Recording... Speak now!")

        audio = recognizer.listen(source)
        print("Recording complete. Saving file...")

        # Save the recording as a WAV file
        with open("test_recording.wav", "wb") as f:
            f.write(audio.get_wav_data())

        print("Recording saved as 'test_recording.wav'. Play it to check.")

if __name__ == "__main__":
    test_recording()
