import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard
import threading

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')

        self.accel = 0.0
        self.steer = 0.0
        self.pressed_keys = set()

        self.publisher = self.create_publisher(String, '/car_controls', 10)
        self.timer = self.create_timer(0.25, self.update_and_publish)  # 4 Hz

        # Start key listener in background
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        self.get_logger().info("KeyboardControlNode started (WASD + space to reset, Q to quit)")

    def clamp(self, val, min_val, max_val):
        return max(min(val, max_val), min_val)

    def on_press(self, key):
        try:
            if hasattr(key, 'char'):
                self.pressed_keys.add(key.char.lower())
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if hasattr(key, 'char'):
                self.pressed_keys.discard(key.char.lower())

            if key.char.lower() == 'q':
                self.get_logger().info("Exiting...")
                rclpy.shutdown()
        except AttributeError:
            pass

    def update_and_publish(self):
        # Obdelava pritisnjenih tipk
        self.accel = 0
        if 'w' in self.pressed_keys:
            self.accel = self.clamp(self.accel + 5, -100, 100)
        if 's' in self.pressed_keys:
            self.accel = self.clamp(self.accel - 5, -100, 100)

        self.steer = 0
        if 'a' in self.pressed_keys:
            self.steer = self.clamp(self.steer - 20, -100, 100)
        if 'd' in self.pressed_keys:
            self.steer = self.clamp(self.steer + 20, -100, 100)

        # if ' ' in self.pressed_keys:
        #     self.accel = 0
        #     self.steer = 0

        msg = String()
        msg.data = f"{self.accel};{self.steer}"
        self.publisher.publish(msg)

        # self.get_logger().info(f"Published: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
