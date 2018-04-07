#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

# import miscellaneous modules
import matplotlib.pyplot as plt
import cv2
import os
import numpy as np
import time
import pandas as pd


def annotate(image, predictions):
    if len(predictions.index) == 0:
        return image
    if time.time() - predictions.iloc[0]['ctime'] > 10:
        return image
    draw = image.copy()

    for idx in range(0, len(predictions.index)):
        c = str(predictions.iloc[idx]['object'])
        s = predictions.iloc[idx]['score']
        b = [predictions.iloc[idx]['lx'],
                 predictions.iloc[idx]['by'],
                 predictions.iloc[idx]['rx'],
                 predictions.iloc[idx]['ty']]

        cv2.rectangle(draw, (b[0], b[1]), (b[2], b[3]), (0, 0, 255), 3)
        caption = "{} {:.3f}".format(c, s)
        cv2.putText(draw, caption, (b[0], b[1] - 10), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 0), 3)
        cv2.putText(draw, caption, (b[0], b[1] - 10), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 255), 2)

    return draw

def image_callback(data):
    print('Recieved image')
    cv2.namedWindow("preview", cv2.WINDOW_NORMAL)
    np_arr = np.fromstring(data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    if os.path.isfile('object_preds.csv'):
        print('annotation exists')
        preds = pd.read_csv('object_preds.csv')
        image_np = annotate(image_np, preds)

    cv2.imshow("preview", image_np)
    cv2.waitKey(1)

    
def mecccamera():
    rospy.init_node('mecccamera', anonymous=True)
    rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, image_callback, queue_size = 1)
    rospy.spin()

if __name__ == '__main__':
    mecccamera()
