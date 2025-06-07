#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import gtsam
import numpy as np
from gtsam.symbol_shorthand import X, L
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from threading import Lock
import math
from collections import defaultdict
import time

class SLAMNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        self.subscription = self.create_subscription(
            String,
            '/cone_detections_3D',
            self.detection_callback,
            10)

        # GTSAM setup
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()
        self.result = gtsam.Values()

        # Initial pose
        self.current_pose_index = 0
        self.graph.add(gtsam.PriorFactorPose2(X(0), gtsam.Pose2(0.0, 0.0, 0.0),
                                              gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))))
        self.initial_estimate.insert(X(0), gtsam.Pose2(0.0, 0.0, 0.0))

        # Landmark tracking
        self.next_landmark_id = 0
        self.landmark_labels = {}
        self.landmarks_by_label = defaultdict(set)

        # Visualization
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.path_line, = self.ax.plot([], [], 'k-', label='Vehicle Path')
        self.color_map = {'B': 'blue', 'Y': 'yellow', 'O': 'orange'}

        # Pose timing
        self.lock = Lock()
        self.last_pose_time = time.time()
        self.pose_update_interval = 0.5  # 2 Hz
        self.odometry = gtsam.Pose2(0.1, 0.0, 0.0)
        self.odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))

        self.get_logger().info("SLAM Node Initialized")

    def detection_callback(self, msg):
        with self.lock:
            now = time.time()
            if now - self.last_pose_time >= self.pose_update_interval:
                self.add_new_pose()
                self.last_pose_time = now

            current_pose = self.initial_estimate.atPose2(X(self.current_pose_index))

            for det in msg.data.strip().split("|"):
                parts = det.strip().split(";")
                if len(parts) != 4:
                    self.get_logger().warn("Malformed detection skipped: " + det)
                    continue
                try:
                    label, x, y, z = parts
                    x, y, z = float(x), float(y), float(z)
                    if x == 0.0 and y == 0.0 and z == 0.0:
                        continue
                except ValueError:
                    self.get_logger().warn("Value error in detection: " + det)
                    continue

                self.get_logger().info(f"Detected: {label} at x={x}, y={y}, z={z}")
                vehicle_point = gtsam.Point2(z / 1000.0, -x / 1000.0)
                world_point = current_pose.transformFrom(vehicle_point)

                candidate_key = None
                min_dist = 2.0
                for key in self.landmarks_by_label[label]:
                    if self.initial_estimate.exists(key):
                        lm_point = self.initial_estimate.atPoint2(key)
                        dist = np.linalg.norm(world_point - lm_point)
                        if dist < min_dist:
                            candidate_key = key
                            min_dist = dist

                if candidate_key is None:
                    landmark_id = self.next_landmark_id
                    self.next_landmark_id += 1
                    candidate_key = L(landmark_id)
                    self.landmark_labels[candidate_key] = label
                    self.landmarks_by_label[label].add(candidate_key)
                    self.initial_estimate.insert(candidate_key, world_point)
                    self.get_logger().info(f"New landmark {label} -> {candidate_key} at {world_point}")
                else:
                    # ✅ Log matched association
                    self.get_logger().info(f"Matched to existing landmark {candidate_key} at distance {min_dist:.2f}m")

                # Add factor
                range_val = np.linalg.norm(vehicle_point)
                bearing = gtsam.Rot2(math.atan2(vehicle_point[1], vehicle_point[0]))
                noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1]))
                factor = gtsam.BearingRangeFactor2D(X(self.current_pose_index), candidate_key,
                                                    bearing, range_val, noise)
                self.graph.add(factor)

            # ✅ Log landmark count after every message
            self.get_logger().info(f"Total unique landmarks: {len(self.landmark_labels)}")

    def add_new_pose(self):
        new_index = self.current_pose_index + 1
        current_pose = self.initial_estimate.atPose2(X(self.current_pose_index))
        new_pose = current_pose.compose(self.odometry)

        self.graph.add(gtsam.BetweenFactorPose2(X(self.current_pose_index), X(new_index),
                                                self.odometry, self.odometry_noise))
        self.initial_estimate.insert(X(new_index), new_pose)
        self.current_pose_index = new_index

        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimate)
        self.result = optimizer.optimize()
        self.initial_estimate = self.result
        self.get_logger().info(f"New pose added at index {new_index}: ({new_pose.x():.3f}, {new_pose.y():.3f}, {new_pose.theta():.3f})")
        self.visualize()

    def visualize(self):
        self.ax.clear()
        self.ax.set_title("GTSAM SLAM Visualization")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True)

        x_vals, y_vals = [], []
        for i in range(self.current_pose_index + 1):
            if self.result.exists(X(i)):
                pose = self.result.atPose2(X(i))
                x_vals.append(pose.x())
                y_vals.append(pose.y())
        self.ax.plot(x_vals, y_vals, 'k-', label='Vehicle Path')

        for label, color in self.color_map.items():
            xs, ys = [], []
            for key in self.landmarks_by_label[label]:
                if self.result.exists(key):
                    pt = self.result.atPoint2(key)
                    xs.append(pt[0])
                    ys.append(pt[1])
            self.ax.scatter(xs, ys, c=color, label=f'{label} cone')

        self.ax.legend()
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = SLAMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
