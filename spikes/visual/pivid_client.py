# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

import asyncio
import websockets
import numpy as np

 
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
  
# allow the camera to warmup
time.sleep(0.1)
   

async def send_frame(frame):
  async with websockets.connect('ws://192.168.1.106:8765') as websocket:
    frame = cv2.imencode('.jpg', frame)[1].tostring()
    await websocket.send(frame)

    greeting = await websocket.recv()
    print("< {}".format(greeting))

frame_count = 0
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
  # grab the raw NumPy array representing the image, then initialize the timestamp
  # and occupied/unoccupied text
  image = frame.array
  frame_count += 1

  # show the frame
  cv2.imshow("Frame", image)
  cv2.waitKey(1)
  if frame_count % 2 == 0:
      try:
        asyncio.get_event_loop().run_until_complete(send_frame(image))
      except:
        print("Dropped frame")

  # clear the stream in preparation for the next frame
  rawCapture.truncate(0)

