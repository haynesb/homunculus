#!/usr/bin/env python

import asyncio
import websockets

import cv2
import numpy as np

#cv2.namedWindow("preview",cv2.WINDOW_NORMAL)

async def hello(websocket, path):
  imgenc = await websocket.recv()
  imgenc2 = np.frombuffer(imgenc,np.uint8)
  imgdec = cv2.imdecode(imgenc2,cv2.IMREAD_UNCHANGED)
#  cv2.imshow("preview", imgdec)
#  cv2.waitKey(1)
  response = "OK"
  await websocket.send(response)
  #print("> {}".format(response))


start_server = websockets.serve(hello, None, 8765)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
