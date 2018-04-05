#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import nltk.corpus
import nltk.tokenize
import nltk.stem.snowball
import string
import yaml
import random
import numpy

# Set up for phrase similarity assessment:
stopwords = nltk.corpus.stopwords.words('english')
stopwords.extend(string.punctuation)
stopwords.append('')

tokenize = nltk.tokenize.word_tokenize

def tokenize_phrase(phrase):
    tokens = [token.lower().strip(string.punctuation) for token in tokenize(phrase) \
    if token.lower().strip(string.punctuation) not in stopwords]
    return tokens

def jaccard_similarity(phrase, tokens):
    """Check if a and b are matches."""
    phrase_tokens = tokenize_phrase(phrase)

    # Calculate Jaccard similarity
    ratio = len(set(phrase_tokens).intersection(tokens)) / float(len(set(phrase_tokens).union(tokens)))
    return ratio

# Load command set:
filestr = file('src/homunculus/resources/commands.yml')
commands = yaml.load(filestr)


def tokenize_cmd(cmd):
    cmdid = cmd[0]
    return [(cmdid,tokenize_phrase(x)) for x in cmd[1]['phrases']]

flatten = lambda l: [item for sublist in l for item in sublist]
cmdtoks = flatten([tokenize_cmd(cmd) for cmd in commands.items()])

def score_phrase(phrase):
    return [jaccard_similarity(phrase, tokens[1]) for tokens in cmdtoks]

def phrase2command(phrase, threshold = 0.5):
    scores = score_phrase(phrase)
    maxindx = flatten(numpy.argwhere(scores == numpy.max(scores)))
    indxcount = len(maxindx)
    if indxcount > 1:
        maxindx = [maxindx[random.randint(0,indxcount-1)]]
    if scores[maxindx[0]] < threshold:
        return None
    return cmdtoks[maxindx[0]][0]



# Initialize publishers:
ttspub = rospy.Publisher('tts', String, queue_size=10)
motionpub = rospy.Publisher('motion', String, queue_size=10)

def stt_callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    cmdid = phrase2command(str(data.data))
    if cmdid is None:
        ttspub.publish("I didn't understand what you said.")

    for action in commands[cmdid]['actions']:
        action = action.split(' ', 1)
        if action[0] == 'motion':
            motionpub.publish(action[1])
        elif action[0] == 'tts':
            ttspub.publish(action[1])
           

def meccontroller():

    rospy.init_node('meccontroller', anonymous=True)

    rospy.Subscriber('stt', String, stt_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    meccontroller()
