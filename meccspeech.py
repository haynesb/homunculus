#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import sys
print('\n'.join(sys.path))

import time
import pygame
import gtts
import hashlib
import os.path

import sys

class MECCSpeech():
    def __init__(self):
        pygame.mixer.init()

    def say(self, ttstext):
        
        ttsfile =  "cache/tts/" + hashlib.md5(ttstext.encode('utf-8')).hexdigest() + ".mp3"
        if not os.path.isfile(ttsfile):
            tts = gtts.gTTS(text=ttstext, lang='en')
            tts.save(ttsfile)
        
        pygame.mixer.music.load(ttsfile)
        time.sleep(0.5)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            time.sleep(0.5)

m = MECCSpeech()


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    m.say(data.data)

def tts_listener():

    rospy.init_node('meccspeech', anonymous=True)

    rospy.Subscriber('tts', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    tts_listener()
