# metrics_node.py

import rclpy
from rclpy.node import Node
from prometheus_client import start_http_server, Gauge
import psutil
import time
import threading

class MetricsNode(Node):
    def __init__(self):
        super().__init__('metrics_node')

        self.cpu_usage = Gauge('ros2_cpu_usage_percent', 'CPU usage in percent')
        self.memory_usage = Gauge('ros2_memory_usage_mb', 'Memory usage in MB')
        self.processed_frames = Gauge('ros2_processed_frames_per_sec', 'Frames processed per second')
        self.recognized_cones = Gauge('ros2_recognized_cones_total', 'Recognized cones per frame')

        start_http_server(8000)
        thread = threading.Thread(target=self.update_metrics)
        thread.daemon = True
        thread.start()

    def update_metrics(self):
        while True:
            self.cpu_usage.set(psutil.cpu_percent())
            self.memory_usage.set(psutil.virtual_memory().used / 1024 / 1024)
            self.processed_frames.set(12.5)
            self.recognized_cones.set(3)
            time.sleep(15)

def main(args=None):
    rclpy.init(args=args)
    node = MetricsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()