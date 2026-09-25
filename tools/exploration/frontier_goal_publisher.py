#!/usr/bin/env python3
"""Lightweight frontier-based goal selection for AeroSLAM exploration.

Subscribes to the registered point cloud and the odometry of the UAV, rasterises
the cloud into a 2D occupancy grid, marks cells along the travelled path as
free, and periodically publishes the centroid of the largest frontier (a free
cell that borders never-observed space) as a 2D Nav Goal.

This is a tutorial-grade baseline explorer: it is intentionally simple,
dependency-free (numpy + rospy only) and works with the same goal interface as
EGO-Planner (`/move_base_simple/goal`).

Usage:
    roslaunch aeroslam_bringup uav_ego_planner.launch gui:=false headless:=true rviz:=false
    rosservice call /enable_motors true
    rosrun aeroslam_bringup position_command_bridge.py         # already in the launch
    python3 tools/exploration/frontier_goal_publisher.py --cooldown 15
"""

import argparse
import math

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class FrontierExplorer(object):
    def __init__(self, args):
        self.args = args
        self.size = int(2 * args.range / args.resolution)
        self.hit = np.zeros((self.size, self.size), dtype=np.uint16)
        self.free = np.zeros((self.size, self.size), dtype=bool)
        self.position = None
        self.last_goal_time = rospy.Time(0)
        self.goals = 0

        self.goal_pub = rospy.Publisher(args.goal_topic, PoseStamped, queue_size=1, latch=True)
        rospy.Subscriber(args.cloud_topic, PointCloud2, self.cloud_cb, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber(args.odom_topic, Odometry, self.odom_cb, queue_size=10)
        rospy.Timer(rospy.Duration(1.0), self.timer_cb)
        rospy.loginfo("frontier_goal_publisher: range %.1f m, resolution %.2f m, cooldown %.1f s",
                      args.range, args.resolution, args.cooldown)

    def to_cell(self, x, y):
        return (int((x + self.args.range) / self.args.resolution),
                int((y + self.args.range) / self.args.resolution))

    def odom_cb(self, msg):
        self.position = msg.pose.pose.position
        cx, cy = self.to_cell(self.position.x, self.position.y)
        if 0 <= cx < self.size and 0 <= cy < self.size:
            self.free[max(0, cx - 2):cx + 3, max(0, cy - 2):cy + 3] = True

    def cloud_cb(self, msg):
        for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if z < self.args.min_height or z > self.args.max_height:
                continue
            cx, cy = self.to_cell(x, y)
            if 0 <= cx < self.size and 0 <= cy < self.size:
                self.hit[cx, cy] = min(self.hit[cx, cy] + 1, 1000)

    def timer_cb(self, _event):
        if self.position is None or self.goals >= self.args.max_goals:
            return
        if (rospy.Time.now() - self.last_goal_time).to_sec() < self.args.cooldown:
            return

        occupied = self.hit >= self.args.occupancy_threshold
        observed = occupied | self.free
        padded_occ = np.copy(occupied)
        for shift in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            padded_occ |= np.roll(occupied, shift, axis=(0, 1))

        frontier = self.free & ~padded_occ & ~observed | (self.free & ~padded_occ)
        unknown_neighbour = ~observed
        for shift in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            unknown_neighbour &= ~np.roll(observed, shift, axis=(0, 1))
        frontier = self.free & ~padded_occ & unknown_neighbour
        if not np.any(frontier):
            # fall back: any free cell that borders unobserved space
            frontier = self.free & ~observed
        if not np.any(frontier):
            rospy.loginfo("frontier_goal_publisher: no frontier found yet")
            return

        cells = np.argwhere(frontier)
        cx, cy = self.position.x, self.position.y
        distances = np.hypot(cells[:, 0] * self.args.resolution - self.args.range - cx,
                             cells[:, 1] * self.args.resolution - self.args.range - cy)
        valid = distances > self.args.min_goal_distance
        if not np.any(valid):
            return
        target = cells[valid][np.argmin(distances[valid])]

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.args.frame_id
        goal.pose.position.x = target[0] * self.args.resolution - self.args.range
        goal.pose.position.y = target[1] * self.args.resolution - self.args.range
        goal.pose.position.z = self.args.altitude
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.last_goal_time = rospy.Time.now()
        self.goals += 1
        rospy.loginfo("frontier_goal_publisher: goal %d -> (%.1f, %.1f)",
                      self.goals, goal.pose.position.x, goal.pose.position.y)


def main():
    parser = argparse.ArgumentParser(description="Frontier-based goal selection for AeroSLAM")
    parser.add_argument("--cloud-topic", default="/cloud_registered")
    parser.add_argument("--odom-topic", default="/Odometry")
    parser.add_argument("--goal-topic", default="/move_base_simple/goal")
    parser.add_argument("--frame-id", default="camera_init")
    parser.add_argument("--range", type=float, default=20.0)
    parser.add_argument("--resolution", type=float, default=0.25)
    parser.add_argument("--altitude", type=float, default=1.5)
    parser.add_argument("--min-height", type=float, default=0.3, help="ignore ground points below this z")
    parser.add_argument("--max-height", type=float, default=2.5)
    parser.add_argument("--occupancy-threshold", type=int, default=3)
    parser.add_argument("--min-goal-distance", type=float, default=2.0)
    parser.add_argument("--cooldown", type=float, default=20.0)
    parser.add_argument("--max-goals", type=int, default=10)
    args, _ = parser.parse_known_args()

    rospy.init_node("frontier_goal_publisher")
    FrontierExplorer(args)
    rospy.spin()


if __name__ == "__main__":
    main()
