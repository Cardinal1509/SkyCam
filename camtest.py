import cv2
import time

STREAM_URL = "http://192.168.1.26/stream"

# Direct OpenCV capture from the HTTP stream
cap = cv2.VideoCapture(STREAM_URL)

if not cap.isOpened():
    print(f"Error: Unable to open video stream at {STREAM_URL}")
    exit(1)

prev_time = time.time()
frame_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame capture failed or stream disconnected.")
        break

    frame_count += 1
    current_time = time.time()
    fps = 1.0 / (current_time - prev_time)
    prev_time = current_time

    cv2.putText(
        frame,
        f"FPS: {fps:.1f}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2,
    )
    cv2.imshow("ESP32-CAM Stream", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()