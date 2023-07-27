import cv2
import numpy as np
import time
import shutil
import os

data_dir = "data/mycam"

vid = cv2.VideoCapture(0)
vid.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
vid.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
  
timestamp_ns = []

ts_file = open(f"{data_dir}/timestamp.txt", "w")

while vid.isOpened():
    ret, frame = vid.read()
    t = time.time()
    t_ns = int(t * 1E9)

    ts_file.write(f"{t_ns}\n")

    left = frame[:, 0:1280]
    right = frame[:, 1280:]

    cv2.imshow("Left-Right", frame)

    cv2.imwrite(f"{data_dir}/cam0_{t_ns}.png", left)
    cv2.imwrite(f"{data_dir}/cam1_{t_ns}.png", right)
    # cv2.imshow("Left", left)
    # cv2.imshow("Right", right)
      
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

ts_file.close()
vid.release()
cv2.destroyAllWindows