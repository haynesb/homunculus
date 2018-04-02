#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import time
import pygame
from gtts import gTTS
import hashlib
import os.path

class MECCSpeech():
    def __init__(self):
        pygame.mixer.init()

    def say(self, ttstext):
        
        ttsfile =  "cache/tts/" + hashlib.md5(ttstext).hexdigest() + ".mp3"
        if not os.path.isfile(ttsfile):
            tts = gTTS(text=ttstext, lang='en')
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

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('tts', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
