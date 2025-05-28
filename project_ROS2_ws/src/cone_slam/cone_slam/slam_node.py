#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import matplotlib.pyplot as plt
import math
import numpy as np

class ConeSLAM(Node):
    def __init__(self):
        super().__init__('cone_slam_node')
        self.subscription = self.create_subscription(
            String,
            '/cone_detection',
            self.listener_callback,
            10)

        self.landmark_map = {}
        self.lock = threading.Lock()

        # EKF state
        self.mu = np.array([0.0, 0.0, 0.0])           # [x, z, theta]
        self.Sigma = np.eye(3) * 1e-3                 # Initial small uncertainty
        self.Q = np.diag([5.0, 5.0, np.deg2rad(2)])   # Motion noise
        self.R = np.diag([50.0, np.deg2rad(5)])       # Measurement noise

        self.path = []

        self.get_logger().info("Cone SLAM node started 🚗📍")

        threading.Thread(target=self.visualize_map, daemon=True).start()

    def listener_callback(self, msg):
        detections = msg.data.split("|")

        # --- EKF Prediction Step (simulated forward motion) ---
        delta_d = 50.0
        delta_theta = 0.0  # no rotation

        theta = self.mu[2]
        dx = delta_d * np.sin(theta)
        dz = delta_d * np.cos(theta)

        self.mu[0] += dx
        self.mu[1] += dz
        self.mu[2] += delta_theta

        G = np.array([
            [1, 0, delta_d * np.cos(theta)],
            [0, 1, -delta_d * np.sin(theta)],
            [0, 0, 1]
        ])

        self.Sigma = G @ self.Sigma @ G.T + self.Q
        self.path.append((self.mu[0], self.mu[1]))

        # --- EKF Correction Step for Each Detection ---
        with self.lock:
            for det in detections:
                try:
                    label, x_str, y_str, z_str = det.split(";")
                    x_l, z_l = int(x_str), int(z_str)

                    # Compute expected measurement
                    x_r, z_r, theta = self.mu
                    dx = x_l - x_r
                    dz = z_l - z_r
                    q = dx**2 + dz**2

                    expected_range = math.sqrt(q)
                    expected_bearing = math.atan2(dx, dz) - theta

                    # Assume measured = expected for now
                    z = np.array([expected_range, expected_bearing])
                    z_hat = np.array([expected_range, expected_bearing])

                    # Jacobian
                    H = np.array([
                        [-dx / expected_range, -dz / expected_range, 0],
                        [ dz / q,             -dx / q,             -1]
                    ])

                    # EKF Update
                    S = H @ self.Sigma @ H.T + self.R
                    K = self.Sigma @ H.T @ np.linalg.inv(S)

                    y = z - z_hat
                    y[1] = (y[1] + np.pi) % (2 * np.pi) - np.pi  # normalize

                    self.mu = self.mu + K @ y
                    self.Sigma = (np.eye(3) - K @ H) @ self.Sigma

                    self.landmark_map[label] = (x_l, z_l)

                except Exception as e:
                    self.get_logger().warn(f"Parse error: {e}")
                    continue

    def visualize_map(self):
        plt.ion()
        fig, ax = plt.subplots()
        while True:
            with self.lock:
                xs = []
                zs = []
                labels = []
                for label, (x, z) in self.landmark_map.items():
                    xs.append(x)
                    zs.append(z)
                    labels.append(label)

                rx, rz, theta = self.mu
                path_copy = self.path[:]

            ax.clear()
            ax.set_title("🧭 Cone Landmark Map (Top-down)")
            ax.set_xlabel("X (mm) → Left/Right")
            ax.set_ylabel("Z (mm) → Forward")
            ax.set_xlim(-2000, 2000)
            ax.set_ylim(0, 6000)
            ax.grid(True)

            ax.scatter(xs, zs, label="Cones")
            for i, txt in enumerate(labels):
                ax.text(xs[i], zs[i] + 100, txt, ha='center', fontsize=9)

            ax.plot(rx, rz, marker=(3, 0, math.degrees(theta) - 90),
                    markersize=15, color='red', label="Robot")

            if path_copy:
                pxs, pzs = zip(*path_copy)
                ax.plot(pxs, pzs, linestyle='--', color='red')

            plt.pause(0.2)

def main(args=None):
    rclpy.init(args=args)
    node = ConeSLAM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

