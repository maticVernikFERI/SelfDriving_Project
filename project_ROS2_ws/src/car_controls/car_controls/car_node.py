import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pigpio
import time

SERVO_PIN = 18
ESC_PIN = 12  # Prilagodi glede na tvojo povezavo

MIN_PULSE = 1500
MAX_PULSE = 2500

class CarControlNode(Node):
    def __init__(self):
        super().__init__('car_node')
        self.pi = pigpio.pi()
        self.subscription = self.create_subscription(
            String,
            '/car_controls',
            self.listener_callback,
            10
        )
        self.get_logger().info('CarControlNode started and subscribed to /car_controls')

    def listener_callback(self, msg):
        try:
            accel_str, steer_str = msg.data.split(';')
            accel = float(accel_str)
            steer = float(steer_str)

            if not (-100 <= accel <= 100 and -100 <= steer <= 100):
                raise ValueError("Values out of range")

            # Map to pulse widths
            self.set_esc(accel)
            self.set_servo(steer)

        except Exception as e:
            self.get_logger().error(f"Invalid message format or value: {msg.data} ({e})")

    def set_servo(self, percent):
        # 0% = full left (500µs), 100% = full right (2500µs)
        pulse = MIN_PULSE + (percent / 100.0) * (MAX_PULSE - MIN_PULSE)
        self.pi.set_servo_pulsewidth(SERVO_PIN, pulse)
        self.get_logger().info(f"Steering: {percent}% -> {pulse:.0f}µs")

    def set_esc(self, percent):
        # Typically: 0% = stop, 100% = full throttle
        pulse = MIN_PULSE + (percent / 100.0) * (MAX_PULSE - MIN_PULSE)
        self.pi.set_servo_pulsewidth(ESC_PIN, pulse)
        self.get_logger().info(f"Throttle: {percent}% -> {pulse:.0f}µs")

    def destroy_node(self):
        self.pi.set_servo_pulsewidth(SERVO_PIN, 0)
        self.pi.set_servo_pulsewidth(ESC_PIN, 0)
        self.pi.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CarControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
