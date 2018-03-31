import time
import pygame
from gtts import gTTS

class MECCSpeech():
    def __init__(self):
        pygame.mixer.init()

    def say(self, ttstext):
        tts = gTTS(text=ttstext, lang='en')
        tts.save("tts.mp3")
        
        pygame.mixer.music.load("tts.mp3")
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            time.sleep(0.5)

m = MECCSpeech()
