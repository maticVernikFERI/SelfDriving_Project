#!/home/ivana/Documents/SelfDriving_Project/project_ROS2_ws/gtsam_venv/bin/python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import gtsam
import numpy as np
import matplotlib.pyplot as plt
import threading

class GTSAMConeSLAM(Node):
    def __init__(self):
        super().__init__('gtsam_cone_slam')
        self.subscription = self.create_subscription(
            String,
            '/cone_detections_3D',
            self.listener_callback,
            10)

        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()
        self.landmark_ids = {}
        self.pose_index = 0
        self.path = []

        self.prev_pose = gtsam.Pose2(0.0, 0.0, 0.0)
        self.prev_key = gtsam.symbol('x', self.pose_index)
        self.initial_estimate.insert(self.prev_key, self.prev_pose)

        # Prior
        prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-3, 1e-3, 1e-2]))
        self.graph.add(gtsam.PriorFactorPose2(self.prev_key, self.prev_pose, prior_noise))

        self.lock = threading.Lock()
        threading.Thread(target=self.visualize, daemon=True).start()
        self.get_logger().info("GTSAM Cone SLAM node started.")

    def listener_callback(self, msg):
        detections = msg.data.split("|")
        self.pose_index += 1
        key = gtsam.symbol('x', self.pose_index)

        # Simulate 10cm forward movement
        new_pose = self.prev_pose.compose(gtsam.Pose2(0.0, 0.1, 0.0))
        odom_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))
        self.graph.add(gtsam.BetweenFactorPose2(self.prev_key, key, gtsam.Pose2(0.0, 0.1, 0.0), odom_noise))
        self.initial_estimate.insert(key, new_pose)
        self.prev_key = key
        self.prev_pose = new_pose
        self.path.append((new_pose.x(), new_pose.y()))

        with self.lock:
            for det in detections:
                try:
                    label, x_str, _, z_str = det.split(";")
                    x, y = float(x_str) / 1000.0, float(z_str) / 1000.0
                    lm_key = gtsam.symbol('l', int(label[-1]))  # Simple hash

                    if lm_key not in self.landmark_ids:
                        self.initial_estimate.insert(lm_key, gtsam.Point2(x, y))
                        self.landmark_ids[lm_key] = (x, y)

                    meas = gtsam.Point2(x, y)
                    meas_noise = gtsam.noiseModel.Isotropic.Sigma(2, 0.1)
                    self.graph.add(gtsam.BearingRangeFactor2D(
                        key, lm_key, new_pose.bearing(meas), new_pose.range(meas), meas_noise))
                except:
                    continue

    def visualize(self):
        plt.ion()
        fig, ax = plt.subplots()
        while True:
            ax.clear()
            ax.set_title("GTSAM Cone SLAM")
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.grid(True)

            with self.lock:
                if self.path:
                    px, py = zip(*self.path)
                    ax.plot(px, py, 'r--', label='Path')

                for _, (x, y) in self.landmark_ids.items():
                    ax.plot(x, y, 'bo')
                    ax.text(x, y + 0.1, 'Cone', ha='center')

            plt.pause(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = GTSAMConeSLAM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
