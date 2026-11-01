import math
import threading
import time
import cv2
import numpy as np
import serial

# --- CONFIGURATION ---
SERIAL_PORT = "COM3"
BAUD_RATE = 115200
STREAM_URL = "http://192.168.1.26/stream"

# Control Setpoints & Deadbands
DEADBAND_X_PX = 18
DEADBAND_Y_PX = 18
DEADBAND_SIZE_PX = 6
TARGET_TAG_PX = 55

# Servo Pan Settings
SERVO_ENABLED = True
SERVO_KP = 0.045
MAX_SERVO_STEP = 6.0
SERVO_DIR = 1  # Press 'i' if servo pans away from tag

# 4-Cable Carriage Settings
WINCH_ENABLED = True
GAZE_ROT_DIR = 1
LAT_GAIN = 0.040
DEPTH_GAIN = 0.18
VERT_GAIN = 0.030


class VideoStream:

  def __init__(self, src):
    self.stream = cv2.VideoCapture(src)
    self.ret, self.frame = self.stream.read()
    self.new_frame = False
    self.stopped = False
    self.lock = threading.Lock()

  def start(self):
    threading.Thread(target=self.update, daemon=True).start()
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
        self.new_frame = True

  def read_new(self):
    with self.lock:
      if not self.new_frame or self.frame is None:
        return None
      self.new_frame = False
      return self.frame.copy()

  def stop(self):
    self.stopped = True
    self.stream.release()


def main():
  global SERVO_DIR, SERVO_ENABLED, WINCH_ENABLED, GAZE_ROT_DIR, TARGET_TAG_PX

  try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.02)
    print(f"Connected to motor controller on {SERIAL_PORT}")
  except Exception as e:
    print(f"Warning: Could not open {SERIAL_PORT}: {e}")
    ser = None

  vs = VideoStream(STREAM_URL).start()

  dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
  parameters = cv2.aruco.DetectorParameters()
  parameters.minMarkerPerimeterRate = 0.015
  detector = cv2.aruco.ArucoDetector(dictionary, parameters)

  servo_angle = 90.0
  last_servo_cmd = 90

  print(
      "Controls: 'q'=Quit | 'h'=Home | 'i'=Invert Servo | 's'=Toggle Servo |"
      " 'm'=Toggle Winch | '[' / ']'=Nudge Servo"
  )

  while True:
    # Print any ACK incoming from ESP32
    if ser and ser.in_waiting:
      ack = ser.readline().decode("utf-8", errors="ignore").strip()
      if len(ack) > 0:
        print(f"ESP32: {ack}")

    frame = vs.read_new()
    if frame is None:
      key = cv2.waitKey(1) & 0xFF
      if key == ord("q"):
        break
      continue

    # Single vertical flip: corrects upside-down camera without chiral mirroring
    frame = cv2.flip(frame, 0)

    h, w = frame.shape[:2]
    cx_frame, cy_frame = w // 2, h // 2

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    tag_count = len(ids) if ids is not None else 0

    if ids is not None:
      pts = corners[0][0].astype(int)
      tag_id = ids.flatten()[0]
      tx = int(pts[:, 0].mean())
      ty = int(pts[:, 1].mean())

      edge_top = np.linalg.norm(pts[0] - pts[1])
      edge_right = np.linalg.norm(pts[1] - pts[2])
      tag_size_px = int((edge_top + edge_right) * 0.5)

      err_x = tx - cx_frame
      err_y = ty - cy_frame
      err_size = TARGET_TAG_PX - tag_size_px

      # Proportional Servo Update
      if SERVO_ENABLED and abs(err_x) > DEADBAND_X_PX:
        step = max(
            -MAX_SERVO_STEP, min(MAX_SERVO_STEP, float(err_x * SERVO_KP))
        )
        servo_angle += SERVO_DIR * step
        servo_angle = max(15.0, min(165.0, servo_angle))

      # Camera Gaze Vectors
      theta = math.radians(GAZE_ROT_DIR * (servo_angle - 90.0))
      fwd_x = -math.sin(theta)
      fwd_y = math.cos(theta)
      right_x = math.cos(theta)
      right_y = math.sin(theta)

      # 3D Cable Displacements (mm)
      if WINCH_ENABLED:
        step_lat = (err_x * LAT_GAIN) if abs(err_x) > DEADBAND_X_PX else 0.0
        step_fwd = (
            (err_size * DEPTH_GAIN) if abs(err_size) > DEADBAND_SIZE_PX else 0.0
        )
        step_vert = (-err_y * VERT_GAIN) if abs(err_y) > DEADBAND_Y_PX else 0.0
      else:
        step_lat, step_fwd, step_vert = 0.0, 0.0, 0.0

      world_dx = (step_lat * right_x) + (step_fwd * fwd_x)
      world_dy = (step_lat * right_y) + (step_fwd * fwd_y)
      world_dz = step_vert

      dx_tenths = int(round(world_dx * 10.0))
      dy_tenths = int(round(world_dy * 10.0))
      dz_tenths = int(round(world_dz * 10.0))
      servo_int = int(round(servo_angle))

      if ser and ser.is_open:
        cmd = f"X{dx_tenths},Y{dy_tenths},Z{dz_tenths},A{servo_int}\n"
        ser.write(cmd.encode("utf-8"))
        last_servo_cmd = servo_int

      for j in range(4):
        cv2.line(frame, tuple(pts[j]), tuple(pts[(j + 1) % 4]), (0, 255, 0), 2)
      cv2.circle(frame, (tx, ty), 5, (0, 0, 255), -1)
      cv2.line(frame, (cx_frame, cy_frame), (tx, ty), (255, 0, 0), 2)

      label = f"ID:{tag_id} Err:({err_x},{err_y}) Pan:{servo_int}deg"
      cv2.putText(
          frame,
          label,
          (max(10, tx - 60), max(40, ty - 15)),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.45,
          (0, 255, 0),
          2,
      )

    cv2.drawMarker(
        frame, (cx_frame, cy_frame), (0, 255, 0), cv2.MARKER_CROSS, 20, 1
    )
    status = (
        f"Tags:{tag_count} | Pan:{int(servo_angle)}deg |"
        f" Winch:{'ON' if WINCH_ENABLED else 'OFF'}"
        f" Servo:{'ON' if SERVO_ENABLED else 'OFF'}"
    )
    cv2.putText(
        frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 2
    )

    cv2.imshow("SkyCam Control Panel", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
      break
    elif key == ord("h"):
      servo_angle = 90.0
      last_servo_cmd = 90
      if ser and ser.is_open:
        ser.write(b"HOME\n")
    elif key == ord("i"):
      SERVO_DIR *= -1
      print(f"SERVO_DIR inverted: {SERVO_DIR}")
    elif key == ord("s"):
      SERVO_ENABLED = not SERVO_ENABLED
      print(f"Servo enabled: {SERVO_ENABLED}")
    elif key == ord("m"):
      WINCH_ENABLED = not WINCH_ENABLED
      print(f"Winches enabled: {WINCH_ENABLED}")
    elif key == ord("["):
      servo_angle = max(15.0, servo_angle - 15.0)
      servo_int = int(round(servo_angle))
      if ser and ser.is_open:
        ser.write(f"X0,Y0,Z0,A{servo_int}\n".encode("utf-8"))
        print(f"Manual command sent: A{servo_int}")
    elif key == ord("]"):
      servo_angle = min(165.0, servo_angle + 15.0)
      servo_int = int(round(servo_angle))
      if ser and ser.is_open:
        ser.write(f"X0,Y0,Z0,A{servo_int}\n".encode("utf-8"))
        print(f"Manual command sent: A{servo_int}")

  if ser and ser.is_open:
    ser.close()
  vs.stop()
  cv2.destroyAllWindows()


if __name__ == "__main__":
  main()