import speech_recognition as sr

def recognize_speech():

    print("Available Microphones:")
    print(sr.Microphone.list_microphone_names())

    r = sr.Recognizer()
    with sr.Microphone(device_index=4) as source:
        print("Say something!")
        audio = r.listen(source)

    try:
        text = r.recognize_google(audio)
        print("You said: " + text)
    except sr.UnknownValueError:
        print("Could not understand audio")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))

if __name__ == "__main__":
    recognize_speech()