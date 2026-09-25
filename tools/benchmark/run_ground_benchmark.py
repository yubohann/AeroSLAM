#!/usr/bin/env python3
"""AeroSLAM ground-navigation benchmark.

Launches a headless ground scenario (`sim_env`), initialises AMCL at the robot
spawn pose, sends a goal and measures navigation outcome:

  success, time to goal, path length (odometry), final distance to goal.

Usage (with the workspace sourced):
    source /opt/ros/noetic/setup.bash
    source <workspace>/devel/setup.bash
    python3 tools/benchmark/run_ground_benchmark.py --goal-x 8 --goal-y 4

Results are written to tools/benchmark/results/ground_XX.json.
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

GOAL_TOLERANCE = 0.5  # m


def make_pose(frame, x, y, z=0.0):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = frame
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = 1.0
    return pose


def main():
    parser = argparse.ArgumentParser(description="AeroSLAM ground benchmark")
    parser.add_argument("--world", default="test2")
    parser.add_argument("--goal-x", type=float, default=8.0)
    parser.add_argument("--goal-y", type=float, default=4.0)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--outdir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "results"))
    parser.add_argument("--index", type=int, default=1)
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    launch = subprocess.Popen(
        ["roslaunch", "sim_env", "config.launch",
         "world:=%s" % args.world, "map:=%s" % args.world,
         "gui:=false", "headless:=true"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

    result = {"world": args.world, "goal": [args.goal_x, args.goal_y],
              "success": False, "time_s": None, "path_length_m": None,
              "final_distance_m": None}
    try:
        rospy.init_node("aeroslam_ground_benchmark", anonymous=True, disable_signals=True)
        deadline = time.time() + 120.0
        while time.time() < deadline:
            topics = {name for name, _ in rospy.get_published_topics()}
            if "/amcl_pose" in topics and "/odom" in topics:
                break
            time.sleep(1.0)
        time.sleep(5.0)

        initial = PoseWithCovarianceStamped()
        initial.header.stamp = rospy.Time.now()
        initial.header.frame_id = "map"
        initial.pose.pose.orientation.w = 1.0
        initial.pose.covariance[0] = 0.25
        initial.pose.covariance[7] = 0.25
        initial.pose.covariance[35] = 0.06
        rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1).publish(initial)
        time.sleep(3.0)

        positions = []

        def odom_cb(msg):
            positions.append((time.time(), msg.pose.pose.position))

        rospy.Subscriber("/odom", Odometry, odom_cb, queue_size=200)
        time.sleep(1.0)

        goal = make_pose("map", args.goal_x, args.goal_y)
        goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
        goal_pub.publish(goal)

        start = time.time()
        while time.time() - start < args.timeout and not rospy.is_shutdown():
            if positions:
                last = positions[-1][1]
                distance = math.hypot(last.x - args.goal_x, last.y - args.goal_y)
                if distance < GOAL_TOLERANCE:
                    result["success"] = True
                    break
            time.sleep(0.5)

        duration = time.time() - start
        path = 0.0
        for (_, a), (_, b) in zip(positions, positions[1:]):
            path += math.hypot(b.x - a.x, b.y - a.y)
        if positions:
            last = positions[-1][1]
            result["final_distance_m"] = round(math.hypot(last.x - args.goal_x, last.y - args.goal_y), 3)
            result["path_length_m"] = round(path, 2)
            result["time_s"] = round(duration, 1)

        print(json.dumps(result, indent=2))
        with open(os.path.join(args.outdir, "ground_%02d.json" % args.index), "w") as handle:
            json.dump(result, handle, indent=2)
    finally:
        os.killpg(os.getpgid(launch.pid), signal.SIGINT)
        try:
            launch.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(launch.pid), signal.SIGKILL)
        for pattern in ("gzserver", "move_base", "amcl", "map_server", "robot_state_publisher", "controller_manager"):
            subprocess.call(["pkill", "-f", pattern])
        time.sleep(5)

    return 0


if __name__ == "__main__":
    sys.exit(main())
