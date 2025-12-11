#!/usr/bin/python3

from http import client
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import paho.mqtt.client as mqtt

class ConeSubscriber(Node):
    def __init__(self):
        super().__init__('cone_subscriber')
        
        # Initialize MQTT client once
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("localhost", 1883)
        self.mqtt_client.loop_start()  # Start background thread for MQTT
        self.get_logger().info('MQTT client connected to localhost:1883')
        
        self.subscription = self.create_subscription(
            String,
            '/cone_detections_3D',
            self.cone_callback,
            10
        )
        self.get_logger().info('Cone subscriber started, listening to /cone_detections_3D')

    def cone_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        self.mqtt_client.publish("demo/topic", f'Detected: {msg.data}')
        self.get_logger().info("Publisher sent a message")
    
    def destroy_node(self):
        # Clean up MQTT connection
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        self.get_logger().info('MQTT client disconnected')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    cone_subscriber = ConeSubscriber()
    
    try:
        rclpy.spin(cone_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        cone_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
