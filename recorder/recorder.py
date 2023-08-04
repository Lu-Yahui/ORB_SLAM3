import cv2
import time
import threading
import base64
import os
import json
import numpy as np

from flask import Flask

Width = 1280
HalfWidth = int(Width / 2)
Height = 480

class StereoCamRecorder(object):
    _instance = None

    @staticmethod
    def get():
        if StereoCamRecorder._instance is None:
            StereoCamRecorder._instance = StereoCamRecorder(device_id=0, base_dir="/home/raspberry/data")
        return StereoCamRecorder._instance

    def __init__(self, device_id, base_dir):
        self.device_id = device_id
        self.base_dir = base_dir
        self.data_dir = None
        self.request_shutdown = False
        self.recording = False
        self.vid = None
        self.void_lock = threading.Lock()
        self.data_queue = []
        self.data_queue_lock = threading.Lock()
        self.data_cv = threading.Condition()
        self.cap_thread = threading.Thread(target=self.capture_data)
        self.write_thread = threading.Thread(target=self.write_data)
        self.latest_image_timestamp = 0
        self.fps = 0.0
        self.latest_image = None
        self.num_recorded = 0
        self.latest_image_lock = threading.Lock()
        self.reset()
        self.vid = None
        self.cap_thread.start()
        self.write_thread.start()

    def num_of_recorded(self):
        return self.num_recorded
    
    def num_of_in_queue(self):
        with self.data_queue_lock:
            return len(self.data_queue)

    def latest_data(self):
        with self.latest_image_lock:
            return self.latest_image, self.latest_image_timestamp
        
    def reset(self):
        self.stop_record()

    def is_camera_opened(self):
        return self.vid is not None and self.vid.isOpened()

    def open_camera(self, device_id):
        self.device_id = device_id
        with self.void_lock:
            self._open_camera(self.device_id)

    def close_camera(self):
        with self.void_lock:
            if self.is_camera_opened():
                self.vid.release()
                self.latest_image_timestamp = 0
                self.latest_image = None
                self.fps = 0.0

    def shutdown(self):
        self.stop_record()
        self.request_shutdown = True
        while self.cap_thread.isAlive():
            time.sleep(0.1)
        if self.vid is not None:
            self.vid.release()
        while self.write_thread.isAlive():
            time.sleep(0.1)
        try:
            self.cap_thread.join()
            self.write_thread.join()
        except Exception as _:
            pass

    def start_record(self):
        self.recording = True

    def stop_record(self):
        self.recording = False
        while True:
            with self.data_queue_lock:
                if len(self.data_queue) == 0:
                    break
                with self.data_cv:
                    self.data_cv.notify()
        self.latest_image_timestamp = 0
        self.latest_image = None
        self.data_dir = None
        self.num_recorded = 0

    def _open_camera(self, device_id):
        vid = None
        try:
            vid = cv2.VideoCapture(device_id)
            vid.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
            vid.set(cv2.CAP_PROP_FRAME_HEIGHT, Height)
            vid.set(cv2.CAP_PROP_FRAME_WIDTH, Width)
            self.vid = vid
        except Exception as _:
            pass
        return vid
    
    def create_new_folder(self, timestamp):
        subfolder = time.strftime('%Y-%m-%d-%H-%M-%S', time.localtime(timestamp))
        data_dir = os.path.join(self.base_dir, subfolder)
        os.makedirs(data_dir)
        return data_dir
    
    def capture_data(self):
        while True:
            if self.request_shutdown:
                break
            if not self.is_camera_opened():
                time.sleep(0.1)
                continue
            ret = False
            frame = None
            t = 0
            with self.void_lock:
                ret, frame = self.vid.read()
                t = time.time()
            if not ret:
                continue
            with self.latest_image_lock:
                self.latest_image = frame
                if self.latest_image_timestamp != 0:
                    self.fps = 1.0 / (t - self.latest_image_timestamp * 1E-9)
                self.latest_image_timestamp = int(t * 1E9)
            if self.recording:
                if self.data_dir is None:
                    self.data_dir = self.create_new_folder(t)
                with self.data_queue_lock:
                    self.data_queue.append((frame, int(t * 1E9)))
                with self.data_cv:
                    self.data_cv.notify()


    def write_data(self):
        while True:
            with self.data_cv:
                self.data_cv.wait()
            if self.request_shutdown:
                break
            frame = None
            timestamp = None
            with self.data_queue_lock:
                if len(self.data_queue) > 0:
                    frame, timestamp = self.data_queue.pop(0)
                    self.num_recorded += 1
            if frame is not None and timestamp is not None:
                left = frame[:, 0:HalfWidth]
                right = frame[:, HalfWidth:]
                cv2.imwrite(f"{self.data_dir}/cam0_{timestamp}.bmp", left)
                cv2.imwrite(f"{self.data_dir}/cam1_{timestamp}.bmp", right)

app = Flask(__name__)

@app.route("/")
def hello_world():
    StereoCamRecorder.get()
    with open("stereo_recorder.html") as f:
        return f.read()

@app.route("/reset")  
def rest():
    print("Reset")
    StereoCamRecorder.get().reset()
    return ""
    
@app.route("/stop")
def stop_recording():
    print("Stop")
    StereoCamRecorder.get().stop_record()
    return f"{StereoCamRecorder.get().recording}"

@app.route("/start")
def start_recording():
    print("Start")
    StereoCamRecorder.get().start_record()
    return f"{StereoCamRecorder.get().recording}"

@app.route("/latest_image/<image_type>")
def latest_image(image_type):
    '''
    image_type: left, right, stereo
    '''
    frame, _ = StereoCamRecorder.get().latest_data()
    img_base64 = ""
    img_res = {"img": img_base64}
    resize_ratio = 0.75
    if frame is not None:
        if image_type == "left":
            img = frame[:, :HalfWidth]
        elif image_type == "right":
            img = frame[:, HalfWidth:]
        elif image_type == "stereo":
            img = frame
        else:
            img = frame
        height, width, _ = img.shape
        img = cv2.resize(img, (int(width * resize_ratio), int(height * resize_ratio)))
        _, buf = cv2.imencode(".jpg", img)
        img_base64 = base64.b64encode(buf).decode("utf-8")
        img_res = {"img": img_base64}
    return img_res

@app.route("/open_camera/<device_id>")
def open_camera(device_id):
    print(device_id)
    StereoCamRecorder.get().open_camera(int(device_id))
    return f"{StereoCamRecorder.get().is_camera_opened()}"

@app.route("/close_camera")
def close_camera():
    StereoCamRecorder.get().close_camera()
    return f"{StereoCamRecorder.get().is_camera_opened()}"

@app.route("/status")
def latest_data():
    _, latest_data_timestamp = StereoCamRecorder.get().latest_data()
    current_session = StereoCamRecorder.get().data_dir
    if current_session is None:
        current_session = "None"
    recorded = StereoCamRecorder.get().num_of_recorded()
    in_queue = StereoCamRecorder.get().num_of_in_queue()
    is_recording = StereoCamRecorder.get().recording
    device_opened = StereoCamRecorder.get().vid is not None and StereoCamRecorder.get().vid.isOpened()
    fps = StereoCamRecorder.get().fps
    status = {"is_recording": is_recording, "current_session": current_session.split("/")[-1], "latest_data_timestamp": latest_data_timestamp, "recorded": recorded, "in_queue": in_queue, "device_opened": device_opened, "fps": fps}
    return json.dumps(status)

if __name__ == "__main__":
    app.run(host="0.0.0.0")