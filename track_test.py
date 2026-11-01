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


# Set to True ONLY if your previous terminal output said "Flipped: True"
MIRROR_FRAME = True


def main():
  stream_url = "http://192.168.1.26/stream"
  vs = VideoStream(stream_url).start()

  dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
  parameters = cv2.aruco.DetectorParameters()
  parameters.minMarkerPerimeterRate = 0.02
  parameters.adaptiveThreshWinSizeMin = 3
  parameters.adaptiveThreshWinSizeMax = 23
  parameters.adaptiveThreshWinSizeStep = 10
  detector = cv2.aruco.ArucoDetector(dictionary, parameters)

  print(f"Connected to {stream_url}. Press 'q' to quit.")

  while True:
    frame = vs.read()
    if frame is None:
      continue

    if MIRROR_FRAME:
      frame = cv2.flip(frame, 1)

    h, w = frame.shape[:2]
    cx_frame, cy_frame = w // 2, h // 2

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    tag_count = len(ids) if ids is not None else 0

    if ids is not None:
      for i, tag_id in enumerate(ids.flatten()):
        pts = corners[i][0].astype(int)
        tx = int(pts[:, 0].mean())
        ty = int(pts[:, 1].mean())
        err_x = tx - cx_frame
        err_y = ty - cy_frame

        for j in range(4):
          cv2.line(
              frame, tuple(pts[j]), tuple(pts[(j + 1) % 4]), (0, 255, 0), 2
          )

        cv2.circle(frame, (tx, ty), 5, (0, 0, 255), -1)
        cv2.line(frame, (cx_frame, cy_frame), (tx, ty), (255, 0, 0), 2)

        label = f"ID:{tag_id} Err:({err_x},{err_y})"
        cv2.putText(
            frame,
            label,
            (tx + 10, ty - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )
        print(f"Tag ID: {tag_id} | Error: ({err_x}, {err_y})")

    cv2.drawMarker(
        frame, (cx_frame, cy_frame), (0, 255, 0), cv2.MARKER_CROSS, 20, 1
    )
    cv2.putText(
        frame,
        f"{w}x{h} | Tags: {tag_count}",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 255),
        2,
    )

    cv2.imshow("ESP32-CAM Tracking Test", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
      break

  vs.stop()
  cv2.destroyAllWindows()


if __name__ == "__main__":
  main()