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

print("Listening...")

while True:
    raw = proc.stdout.read(WIDTH * HEIGHT * 3)
    if not raw:
        print("No data yet...")
        continue

    frame = np.frombuffer(raw, dtype=np.uint8)
    frame = frame.reshape((HEIGHT, WIDTH, 3))

    print("Frame received:", frame.shape)
