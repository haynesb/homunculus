#!/usr/bin/env python

import rospy
from std_msgs.msg import String

from snowboy import snowboydecoder
import sys
import signal

import speech_recognition as sr

r = sr.Recognizer()


interrupted = False


def signal_handler(signal, frame):
    global interrupted
    interrupted = True


def interrupt_callback():
    global interrupted
    return interrupted

if len(sys.argv) == 1:
    print("Error: need to specify model name")
    print("Usage: python demo.py your.model")
    sys.exit(-1)

model = sys.argv[1]

# capture SIGINT signal, e.g., Ctrl+C
signal.signal(signal.SIGINT, signal_handler)

detector = snowboydecoder.HotwordDetector(model, sensitivity=0.5)
print('Listening... Press Ctrl+C to exit')

motionpub = rospy.Publisher('motion', String, queue_size=10)
sttpub = rospy.Publisher('stt', String, queue_size=10)
rospy.init_node('mecclisten', anonymous=True)

def google_stt(fname):
    motionpub.publish("EYECOLOR 0 7 0 0")
    with sr.AudioFile(fname) as source:
        audio = r.record(source)  # read the entire audio file
    try:
        # for testing purposes, we're just using the default API key
        # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
        # instead of `r.recognize_google(audio)`
        stt_result = r.recognize_google(audio)
        print("Google Speech Recognition thinks you said " + stt_result)
        rospy.loginfo(stt_result)
        sttpub.publish(stt_result)
        motionpub.publish("EYECOLOR 0 0 7 0")


    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))

# main loop
detector.start(detected_callback=snowboydecoder.play_audio_file,
               interrupt_check=interrupt_callback,
               sleep_time=0.03,
               audio_recorder_callback=google_stt,
               silent_count_threshold=1,
               recording_timeout=20,
               audio_save_path="/home/bhaynes/catkin_ws/cache/stt/")

detector.terminate()
