import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from prometheus_client import start_http_server, Gauge
import psutil
import time
import threading

# public_blue = 0
# public_yellow = 0
# public_orange = 0
# public_total = 0

class MetricsNode(Node):
    def __init__(self):
        super().__init__('metrics_node')

        # Prometheus metrike
        self.cpu_usage = Gauge('ros2_cpu_usage_percent', 'CPU usage in percent')
        self.memory_usage = Gauge('ros2_memory_usage_mb', 'Memory usage in MB')
        self.cones_blue = Gauge('ros2_cones_blue_count', 'Number of blue cones per frame')
        self.cones_yellow = Gauge('ros2_cones_yellow_count', 'Number of yellow cones per frame')
        self.cones_orange = Gauge('ros2_cones_orange_count', 'Number of orange cones per frame')
        self.total_cones = Gauge('ros2_cones_total_count', 'Total number of cones per frame')
        self.processed_frames = Gauge('ros2_processed_frames_per_sec', 'Number of processed frames per second')

        self.frame_counter = 0

        # Začetek Prometheus HTTP strežnika
        start_http_server(8000)

        # Subscriber na /cone_detections_3D
        self.subscription = self.create_subscription(
            String,
            '/cone_detections_3D',
            self.detection_callback,
            10
        )

        # Ozadje za sistemske metrike + FPS
        thread = threading.Thread(target=self.update_metrics)
        thread.daemon = True
        thread.start()

    def detection_callback(self, msg):
        self.frame_counter += 1

        # Primer: "B;123;456;789|Y;321;654;987|O;..."
        detections = msg.data.strip().split('|')
        count_blue = 0
        count_yellow = 0
        count_orange = 0

        for det in detections:
            parts = det.split(';')
            if len(parts) >= 1:
                label = parts[0].strip()
                if label == 'B':
                    count_blue += 1
                elif label == 'Y':
                    count_yellow += 1
                elif label == 'O':
                    count_orange += 1

        total = count_blue + count_yellow + count_orange

        # Posodobi Prometheus metrike
        self.cones_blue.set(count_blue)
        self.cones_yellow.set(count_yellow)
        self.cones_orange.set(count_orange)
        self.total_cones.set(total)
        # global public_blue, public_yellow, public_orange, public_total
        # public_blue = count_blue
        # public_yellow = count_yellow
        # public_orange = count_orange
        # public_total = total

    def update_metrics(self):
        while True:
            # Sistem
            self.cpu_usage.set(psutil.cpu_percent())
            self.memory_usage.set(psutil.virtual_memory().used / 1024 / 1024)

            # FPS
            self.processed_frames.set(self.frame_counter / 5)
            self.frame_counter = 0
            
            # self.cones_blue.set(public_blue)
            # self.cones_yellow.set(public_yellow)
            # self.cones_orange.set(public_orange)
            # self.total_cones.set(public_total)

            time.sleep(5)

def main(args=None):
    rclpy.init(args=args)
    node = MetricsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
