#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import pickle

# import miscellaneous modules
import matplotlib.pyplot as plt
import cv2
import os
import numpy as np
import time

def annotate(image, predictions):
    draw = image.copy()

    for prediction in predictions:
        c = prediction[0]
        s = prediction[1]
        b = prediction[2]

        cv2.rectangle(draw, (b[0], b[1]), (b[2], b[3]), (0, 0, 255), 3)
        caption = "{} {:.3f}".format(c, s)
        cv2.putText(draw, caption, (b[0], b[1] - 10), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 0), 3)
        cv2.putText(draw, caption, (b[0], b[1] - 10), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 255), 2)

    return draw

def image_callback(data):
    print('Recieved image')
    cv2.namedWindow("preview",cv2.WINDOW_NORMAL)
    np_arr = np.fromstring(data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    if os.path.isfile('visionannot.pkl'): 
        visionannot = pickle.load(open('visionannot.pkl','rb'))
        image_np = annotate(image_np,visionannot[1])

    cv2.imshow("preview", image_np)
    cv2.waitKey(1)

def visionannot_callback(data):
    print('Received annot')
    annotfile = open('visionannot.pkl', 'wb')
    print(pickle.dumps(data.data))
    pickle.dump(annotfile, data.data)
    annotfile.close()

    
def mecccamera():
    rospy.init_node('mecccamera', anonymous=True)
    rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, image_callback, queue_size = 1)
    rospy.Subscriber('visionannot', String, visionannot_callback)
    rospy.spin()

if __name__ == '__main__':
    mecccamera()
