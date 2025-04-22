import cv2
import depthai as dai
import time
import os

# ==== SETTINGS ====
SAVE_DIR = "captured_images"
FPS = 2  # Frames per second
DELAY = 1.0 / FPS

# ==== SETUP ====
if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR)

# Create pipeline
pipeline = dai.Pipeline()
cam_rgb = pipeline.create(dai.node.ColorCamera)
xout = pipeline.create(dai.node.XLinkOut)

cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
cam_rgb.setFps(30)  # OAK-D can do 30, we’ll just save fewer

xout.setStreamName("video")
cam_rgb.video.link(xout.input)

# ==== STARTING DEVICE ====
with dai.Device(pipeline) as device:
    video_queue = device.getOutputQueue(name="video", maxSize=4, blocking=False)
    print("Started capturing at ~2 FPS. Press CTRL+C to stop.")

    try:
        frame_count = 0
        while True:
            frame = video_queue.get().getCvFrame()
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"{SAVE_DIR}/frame_{frame_count}_{timestamp}.jpg"
            cv2.imwrite(filename, frame)
            print(f"Saved {filename}")
            frame_count += 1
            time.sleep(DELAY)

    except KeyboardInterrupt:
        print("\nStopped.")
