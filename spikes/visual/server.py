#!/usr/bin/env python

import asyncio
import websockets

import cv2
import numpy as np
import models.retinanet as retnet

import vlc
import time
from gtts import gTTS


cv2.namedWindow("preview",cv2.WINDOW_NORMAL)

async def hello(websocket, path):
    imgenc = await websocket.recv()
    imgenc2 = np.frombuffer(imgenc,np.uint8)
    imgdec = cv2.imdecode(imgenc2,cv2.IMREAD_UNCHANGED)
    pred = retnet.predict(imgdec)
    draw = retnet.annotate(imgdec, pred)
    print(pred)
    cv2.imshow("preview", draw)
    objects = ', '.join([x[0] for x in pred])

    tts = gTTS(text='I see a ' + objects, lang='en')
    tts.save("see.mp3")
    p = vlc.MediaPlayer("see.mp3")
    p.play()


    cv2.waitKey(1)
    response = "OK"
    await websocket.send(response)
    #print("> {}".format(response))


start_server = websockets.serve(hello, None, 8765)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
