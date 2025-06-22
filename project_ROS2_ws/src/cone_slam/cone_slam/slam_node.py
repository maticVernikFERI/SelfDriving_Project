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

        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()
        self.result = gtsam.Values()

        self.current_pose_index = 0
        self.graph.add(gtsam.PriorFactorPose2(
            X(0), gtsam.Pose2(0.0, 0.0, 0.0),
            gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
        ))
        self.initial_estimate.insert(X(0), gtsam.Pose2(0.0, 0.0, 0.0))

        self.next_landmark_id = 0
        self.landmark_labels = {}
        self.landmarks_by_label = defaultdict(set)

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.path_line, = self.ax.plot([], [], 'k-', label='Vehicle Path')
        self.color_map = {'B': 'blue', 'Y': 'yellow', 'O': 'orange'}

        self.lock = Lock()
        self.last_pose_time = time.time()
        self.pose_update_interval = 0.5  # seconds

        self.get_logger().info("SLAM Node Initialized")

    def detection_callback(self, msg):
        with self.lock:
            now = time.time()

            blue_cones = []
            yellow_cones = []

            current_pose = self.initial_estimate.atPose2(X(self.current_pose_index))

            for det in msg.data.strip().split("|"):
                parts = det.strip().split(";")
                if len(parts) != 4:
                    continue
                try:
                    label, x, y, z = parts
                    x, y, z = float(x), float(y), float(z)
                    if x == 0.0 and y == 0.0 and z == 0.0:
                        continue
                except ValueError:
                    continue

                # Convert camera to vehicle frame, then to world
                vehicle_point = gtsam.Point2(z / 1000.0, -x / 1000.0)
                world_point = current_pose.transformFrom(vehicle_point)

                if label == 'B':
                    blue_cones.append(world_point)
                elif label == 'Y':
                    yellow_cones.append(world_point)

                # Data association
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

                range_val = np.linalg.norm(vehicle_point)
                bearing = gtsam.Rot2(math.atan2(vehicle_point[1], vehicle_point[0]))
                noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1]))
                self.graph.add(gtsam.BearingRangeFactor2D(X(self.current_pose_index), candidate_key, bearing, range_val, noise))

            # If enough time has passed and cones are detected, estimate pose from cones
            if now - self.last_pose_time >= self.pose_update_interval:
                self.last_pose_time = now
                midpoint = self.estimate_midpoint_from_cones(blue_cones, yellow_cones)
                if midpoint is not None:
                    self.add_new_pose_from_midpoint(midpoint)

    def estimate_midpoint_from_cones(self, blues, yellows):
        if not blues or not yellows:
            return None
        min_dist = float('inf')
        best_pair = None
        for b in blues:
            for y in yellows:
                d = np.linalg.norm(b - y)
                if d < min_dist:
                    min_dist = d
                    best_pair = (b, y)
        if best_pair:
            return 0.5 * (best_pair[0] + best_pair[1])
        return None

    def add_new_pose_from_midpoint(self, midpoint):
        new_index = self.current_pose_index + 1
        prev_pose = self.initial_estimate.atPose2(X(self.current_pose_index))
        dx = midpoint[0] - prev_pose.x()
        dy = midpoint[1] - prev_pose.y()
        theta = math.atan2(dy, dx)
        new_pose = gtsam.Pose2(midpoint[0], midpoint[1], theta)

        relative_motion = prev_pose.between(new_pose)
        self.graph.add(gtsam.BetweenFactorPose2(X(self.current_pose_index), X(new_index),
                                                relative_motion,
                                                gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))))
        self.initial_estimate.insert(X(new_index), new_pose)
        self.current_pose_index = new_index

        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimate)
        self.result = optimizer.optimize()
        self.initial_estimate = self.result
        self.visualize()

    def visualize(self):
        self.ax.clear()
        self.ax.set_title("GTSAM SLAM Visualization")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True)

        # Plot path
        x_vals, y_vals = [], []
        for i in range(self.current_pose_index + 1):
            if self.result.exists(X(i)):
                pose = self.result.atPose2(X(i))
                x_vals.append(pose.x())
                y_vals.append(pose.y())
        self.ax.plot(x_vals, y_vals, 'k-', label='Vehicle Path')

        # Plot cones
        for label, color in self.color_map.items():
            xs, ys = [], []
            for key in self.landmarks_by_label[label]:
                if self.result.exists(key):
                    pt = self.result.atPoint2(key)
                    xs.append(pt[0])
                    ys.append(pt[1])
            self.ax.scatter(xs, ys, c=color, label=f'{label} cones')

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

