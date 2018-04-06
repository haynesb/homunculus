#!/usr/bin/env python
import rospy
from std_msgs.msg import String

# import keras
import keras

# import keras_retinanet
from keras_retinanet.models.resnet import custom_objects
from keras_retinanet.utils.image import read_image_bgr, preprocess_image, resize_image
from keras_retinanet.utils.visualization import draw_box, draw_caption
from keras_retinanet.utils.colors import label_color


# import miscellaneous modules
import matplotlib.pyplot as plt
import cv2
import os
import numpy as np
import time

# set tf backend to allow memory to grow, instead of claiming everything
import tensorflow as tf

def get_session():
  config = tf.ConfigProto()
  config.gpu_options.allow_growth = True
  return tf.Session(config=config)

# use this environment flag to change which GPU to use
#os.environ["CUDA_VISIBLE_DEVICES"] = "1"

# set the modified tf session as backend in keras
keras.backend.tensorflow_backend.set_session(get_session())

# adjust this to point to your downloaded/trained model
model_path = os.path.join('src', 'homunculus', 'model', 'resnet50_coco_best_v2.0.2.h5')


# load retinanet model
model = keras.models.load_model(model_path, custom_objects=custom_objects)

# load label to names mapping for visualization purposes
labels_to_names = {0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane', 5: 'bus', 6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light', 10: 'fire hydrant', 11: 'stop sign', 12: 'parking meter', 13: 'bench', 14: 'bird', 15: 'cat', 16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow', 20: 'elephant', 21: 'bear', 22: 'zebra', 23: 'giraffe', 24: 'backpack', 25: 'umbrella', 26: 'handbag', 27: 'tie', 28: 'suitcase', 29: 'frisbee', 30: 'skis', 31: 'snowboard', 32: 'sports ball', 33: 'kite', 34: 'baseball bat', 35: 'baseball glove', 36: 'skateboard', 37: 'surfboard', 38: 'tennis racket', 39: 'bottle', 40: 'wine glass', 41: 'cup', 42: 'fork', 43: 'knife', 44: 'spoon', 45: 'bowl', 46: 'banana', 47: 'apple', 48: 'sandwich', 49: 'orange', 50: 'broccoli', 51: 'carrot', 52: 'hot dog', 53: 'pizza', 54: 'donut', 55: 'cake', 56: 'chair', 57: 'couch', 58: 'potted plant', 59: 'bed', 60: 'dining table', 61: 'toilet', 62: 'tv', 63: 'laptop', 64: 'mouse', 65: 'remote', 66: 'keyboard', 67: 'cell phone', 68: 'microwave', 69: 'oven', 70: 'toaster', 71: 'sink', 72: 'refrigerator', 73: 'book', 74: 'clock', 75: 'vase', 76: 'scissors', 77: 'teddy bear', 78: 'hair drier', 79: 'toothbrush'}


def predict(image):

    # preprocess image for network
    image = preprocess_image(image)
    image, scale = resize_image(image)

    # process image
    start = time.time()
    _, _, boxes, nms_classification = model.predict_on_batch(np.expand_dims(image, axis=0))
    print("processing time: ", time.time() - start)

    # compute predicted labels and scores
    predicted_labels = np.argmax(nms_classification[0, :, :], axis=1)
    scores = nms_classification[0, np.arange(nms_classification.shape[1]), predicted_labels]

    # correct for image scale
    boxes /= scale

    # summarize detected
    predictions = list()
    for label, score, box in zip(predicted_labels, scores, boxes[0]):
        if score < 0.5:
            continue
        b = box.astype(int)
        c = labels_to_names[label]
        s = score
        predictions.append((c, s, b))
    
    return predictions


def annotate(image, predictions):
    draw = image.copy()
    #draw = cv2.cvtColor(draw, cv2.COLOR_BGR2RGB)

    for prediction in predictions:
        c = prediction[0]
        s = prediction[1]
        b = prediction[2]

        cv2.rectangle(draw, (b[0], b[1]), (b[2], b[3]), (0, 0, 255), 3)
        caption = "{} {:.3f}".format(c, s)
        cv2.putText(draw, caption, (b[0], b[1] - 10), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 0), 3)
        cv2.putText(draw, caption, (b[0], b[1] - 10), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 255), 2)

    return draw

# load image
image = read_image_bgr('src/homunculus/data/000000008021.jpg')
cv2.namedWindow("preview",cv2.WINDOW_NORMAL)

preds = predict(image)
animg = annotate(image,preds)
cv2.imshow("preview", animg)
cv2.waitKey(0)


# Initialize publishers:
ttspub = rospy.Publisher('tts', String, queue_size=10)
motionpub = rospy.Publisher('motion', String, queue_size=10)

def vision_callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    objects = [pred[0] for pred in preds]
    ttspub.publish("I see a " + objects.join(', '))


def meccvision():

    rospy.init_node('meccontroller', anonymous=True)

    rospy.Subscriber('vision', String, vision_callback)

    ttspub.publish("Vision system online")
    print (preds)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    meccvision()
