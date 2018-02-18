#!/usr/bin/env python

import asyncio
import websockets
import time

import cv2
import numpy as np

vc = cv2.VideoCapture(0)

if vc.isOpened(): # try to get the first frame
    rval, frame = vc.read()
else:
    rval = False


async def hello():
  rval, frame = vc.read()
  if rval:
    async with websockets.connect('ws://192.168.1.109:8765') as websocket:
      frame = cv2.imencode('.jpg', frame)[1].tostring()
      await websocket.send(frame)

      greeting = await websocket.recv()
      print("< {}".format(greeting))
  else:
    print("Frame not ready")

counter = 0
while True:
  counter += 1
  time.sleep(0.033)
  rval, frame = vc.read()
  try:
    print("Issuing frame")
    asyncio.get_event_loop().run_until_complete(hello())
  except:
    print("Could not write to socket")

