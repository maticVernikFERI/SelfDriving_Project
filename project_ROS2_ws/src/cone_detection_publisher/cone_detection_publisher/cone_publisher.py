#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pathlib import Path
import cv2
import depthai as dai
import numpy as np
import time
import threading

class ConePublisher(Node):
    def __init__(self):
        super().__init__('cone_publisher')
        self.publisher_ = self.create_publisher(String, '/cone_detections_3D', 10)
        self.nnBlobPath = str((Path(__file__).parent / Path('./yolov11n_coneDetection_openvino_2022.1_5shave.blob')).resolve().absolute())
        self.labelMap = ["B", "O", "Y"]  # B = blue, O = orange, Y = yellow
        self.syncNN = True
        self.running = True

        if not Path(self.nnBlobPath).exists():
            raise FileNotFoundError(f'Model not found at {self.nnBlobPath}')

        self.pipeline = self.create_pipeline()
        self.run_thread = threading.Thread(target=self.run)
        self.run_thread.start()

    def create_pipeline(self):
        pipeline = dai.Pipeline()

        camRgb = pipeline.create(dai.node.ColorCamera)
        spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)

        xoutRgb = pipeline.create(dai.node.XLinkOut)
        xoutNN = pipeline.create(dai.node.XLinkOut)

        xoutRgb.setStreamName("rgb")
        xoutNN.setStreamName("detections")

        camRgb.setPreviewSize(640, 640)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())
        stereo.setSubpixel(True)

        spatialDetectionNetwork.setBlobPath(self.nnBlobPath)
        spatialDetectionNetwork.setConfidenceThreshold(0.4)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(5000)

        spatialDetectionNetwork.setNumClasses(len(self.labelMap))
        spatialDetectionNetwork.setCoordinateSize(4)
        spatialDetectionNetwork.setIouThreshold(0.5)

        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        camRgb.preview.link(spatialDetectionNetwork.input)
        if self.syncNN:
            spatialDetectionNetwork.passthrough.link(xoutRgb.input)
        else:
            camRgb.preview.link(xoutRgb.input)

        spatialDetectionNetwork.out.link(xoutNN.input)
        stereo.depth.link(spatialDetectionNetwork.inputDepth)

        return pipeline

    def run(self):
        with dai.Device(self.pipeline) as device:
            previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
            
            # start_time = time.monotonic()
            # frame = 0

            while rclpy.ok() and self.running:
                # frame += 1
                # inPreview = previewQueue.get()
                inDet = detectionNNQueue.get()
                # frame = inPreview.getCvFrame()
                detections = inDet.detections

                # if frame == 10:
                #     current_time = time.monotonic()
                #     dps = 10 / (current_time - start_time)
                #     print(f"FPS: {dps:.2f}")
                #     start_time = current_time
                #     frame = 0

                # Build string to publish
                cone_msgs = []
                for detection in detections:
                    label = self.labelMap[detection.label] if detection.label < len(self.labelMap) else str(detection.label)
                    x = int(detection.spatialCoordinates.x)
                    y = int(detection.spatialCoordinates.y)
                    z = int(detection.spatialCoordinates.z)
                    cone_msgs.append(f"{label};{x};{y};{z}")

                message = String()
                message.data = "|".join(cone_msgs)
                self.publisher_.publish(message)

                # Optional: show the image for debug
                # cv2.imshow("rgb", frame)
                # if cv2.waitKey(1) == ord('q'):
                #     self.running = False
                #     break

def main(args=None):
    rclpy.init(args=args)
    node = ConePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
