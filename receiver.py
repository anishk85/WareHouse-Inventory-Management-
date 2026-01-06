import cv2
import subprocess
import numpy as np

WIDTH = 640
HEIGHT = 480

cmd = [
    "ffmpeg",
    "-loglevel", "quiet",
    "-i", "tcp://0.0.0.0:8000?listen",
    "-f", "rawvideo",
    "-pix_fmt", "bgr24",
    "-"
]

proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)

print("Receiver started... waiting for video...")

while True:
    raw = proc.stdout.read(WIDTH * HEIGHT * 3)
    if not raw:
        continue

    frame = np.frombuffer(raw, dtype=np.uint8).reshape((HEIGHT, WIDTH, 3))
    cv2.imshow("RPi4 Feed", frame)
    if cv2.waitKey(1) == 27:
        break
