#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math

class SimpleDriver(Node):
    def __init__(self):
        super().__init__('simple_driver')
        self.subscription = self.create_subscription(
            String,
            '/cone_detections_3D',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(String, '/car_controls', 10)

    def listener_callback(self, msg):
        blue_cones = []
        yellow_cones = []

        if msg.data:
            detections = msg.data.split("|")
            for det in detections:
                try:
                    label, x, y, z = det.split(";")
                    x = int(x)
                    z = int(z)
                    if x == 0 and z == 0:
                        continue
                    if label == "B":
                        blue_cones.append((x, z))
                    elif label == "Y":
                        yellow_cones.append((x, z))
                except ValueError:
                    continue

        # If no cones at all → stop and go straight
        if not blue_cones and not yellow_cones:
            accel = 0
            steer = 0
            self.get_logger().warn("No cones detected — failsafe activated.")
        else:
            steer = self.calculate_steering(blue_cones, yellow_cones)
            accel = 8

        msg_out = String()
        msg_out.data = f"{accel};{steer}"
        self.publisher.publish(msg_out)

        self.get_logger().info(f"Published: {msg_out.data}")

    def calculate_steering(self, blue, yellow):
        if blue and not yellow:
            return -100  # full right
        elif yellow and not blue:
            return 100  # full left

        closest_blue = min(blue, key=lambda c: c[1])
        closest_yellow = min(yellow, key=lambda c: c[1])

        mid_x = (closest_blue[0] + closest_yellow[0]) / 2
        mid_z = (closest_blue[1] + closest_yellow[1]) / 2

        # Cross product with heading vector (0, 1): -mid_x
        max_range = 1000.0
        steer = int(max(min(-mid_x / max_range * 100, 100), -100))
        return -steer

def main(args=None):
    rclpy.init(args=args)
    node = SimpleDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
