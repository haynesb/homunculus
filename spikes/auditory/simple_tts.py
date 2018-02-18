import vlc
import time
from gtts import gTTS

tts = gTTS(text='No, that is boring.', lang='en')
tts.save("hello.mp3")
p = vlc.MediaPlayer("hello.mp3")
p.play()

while p.is_playing():
    time.sleep(1)
