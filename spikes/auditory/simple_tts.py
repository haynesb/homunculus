import time
import pygame
from gtts import gTTS

tts = gTTS(text='No, that is boring.', lang='en')
tts.save("hello.mp3")

pygame.mixer.init()
pygame.mixer.music.load("hello.mp3")
pygame.mixer.music.play()
while True:
    if not pygame.mixer.music.get_busy():
        print("Playing")
        pygame.mixer.music.play()
    time.sleep(0.5)
