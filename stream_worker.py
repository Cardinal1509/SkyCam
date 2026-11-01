import threading
import time
import cv2


class VideoStream:

  def __init__(self, src="http://192.168.1.26/stream"):
    self.stream = cv2.VideoCapture(src)
    self.ret, self.frame = self.stream.read()
    self.stopped = False
    self.lock = threading.Lock()

  def start(self):
    t = threading.Thread(target=self.update, args=(), daemon=True)
    t.start()
    return self

  def update(self):
    while not self.stopped:
      if not self.stream.isOpened():
        time.sleep(0.01)
        continue
      ret, frame = self.stream.read()
      if not ret:
        continue
      with self.lock:
        self.frame = frame

  def read(self):
    with self.lock:
      return self.frame.copy() if self.frame is not None else None

  def stop(self):
    self.stopped = True
    self.stream.release()