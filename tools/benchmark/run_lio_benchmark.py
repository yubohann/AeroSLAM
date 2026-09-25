#!/usr/bin/env python3
"""AeroSLAM benchmark: LiDAR-inertial odometry against ground truth.

Launches the headless UAV scenario (Livox + FAST-LIO), commands a waypoint
flight through the hector position controller, records FAST-LIO odometry and
Gazebo ground truth, and reports rates, path length and position drift.

Drift is computed after nearest-neighbour pairing of ground-truth samples to
odometry timestamps, both raw (offset-aligned) and after 2D rigid alignment
(Procrustes), which removes frame-heading conventions.

Usage (with the workspace sourced):
    source /opt/ros/noetic/setup.bash
    source <workspace>/devel/setup.bash
    python3 tools/benchmark/run_lio_benchmark.py --duration 45 --runs 1

Results are written to tools/benchmark/results/ as JSON plus a printed table.
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from hector_uav_msgs.srv import EnableMotors
from nav_msgs.msg import Odometry

WAYPOINTS = [(0.0, 0.0, 1.5), (4.0, 0.0, 1.5), (4.0, 4.0, 1.5), (0.0, 4.0, 1.5)]


class Recorder(object):
    def __init__(self):
        self.odom = []
        self.truth = []
        rospy.Subscriber("/Odometry", Odometry, self._odom_cb, queue_size=200)
        rospy.Subscriber("/ground_truth/state", Odometry, self._truth_cb, queue_size=200)

    def _odom_cb(self, msg):
        self.odom.append((msg.header.stamp.to_sec(), msg.pose.pose.position))

    def _truth_cb(self, msg):
        self.truth.append((msg.header.stamp.to_sec(), msg.pose.pose.position))


def enable_motors():
    for service in ("/quadrotor/enable_motors", "enable_motors", "/enable_motors"):
        try:
            rospy.wait_for_service(service, timeout=5.0)
        except rospy.ROSException:
            continue
        try:
            response = rospy.ServiceProxy(service, EnableMotors)(True)
            if response.success:
                print("motors enabled via %s" % service)
                return True
        except rospy.ServiceException:
            continue
    print("warning: could not enable motors")
    return False


def fly_waypoints(duration):
    pub = rospy.Publisher("command/pose", PoseStamped, queue_size=1)
    rate = rospy.Rate(50)
    start = time.time()
    index = 0
    while not rospy.is_shutdown() and time.time() - start < duration:
        x, y, z = WAYPOINTS[index % len(WAYPOINTS)]
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "world"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pub.publish(pose)
        if time.time() - start > 3.0 * (index + 1):
            index += 1
        rate.sleep()


def wait_for_topics(timeout):
    needed = {"/Odometry", "/ground_truth/state"}
    deadline = time.time() + timeout
    while time.time() < deadline and not rospy.is_shutdown():
        published = {name for name, _ in rospy.get_published_topics()}
        if needed.issubset(published):
            time.sleep(3.0)
            return True
        time.sleep(0.5)
    return False


def pair_by_time(odom, truth):
    """Nearest-neighbour pairing of truth samples to odometry timestamps."""
    pairs = []
    j = 0
    for t_odom, p_odom in odom:
        while j + 1 < len(truth) and abs(truth[j + 1][0] - t_odom) < abs(truth[j][0] - t_odom):
            j += 1
        pairs.append((p_odom, truth[j][1]))
    return pairs


def summarise(recorder):
    def stats(samples):
        if len(samples) < 2:
            return 0.0, 0.0
        t0, t1 = samples[0][0], samples[-1][0]
        rate = (len(samples) - 1) / (t1 - t0) if t1 > t0 else 0.0
        length = 0.0
        for (_, a), (_, b) in zip(samples, samples[1:]):
            length += math.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2 + (b.z - a.z) ** 2)
        return rate, length

    odom_rate, odom_length = stats(recorder.odom)
    _, truth_length = stats(recorder.truth)

    pairs = pair_by_time(recorder.odom, recorder.truth) if recorder.odom and recorder.truth else []

    rmse = float("nan")
    final_drift = float("nan")
    max_drift = float("nan")
    if pairs:
        o0, t0 = pairs[0]
        errors = []
        for o, t in pairs:
            dx = (o.x - o0.x) - (t.x - t0.x)
            dy = (o.y - o0.y) - (t.y - t0.y)
            dz = (o.z - o0.z) - (t.z - t0.z)
            errors.append(math.sqrt(dx * dx + dy * dy + dz * dz))
        rmse = math.sqrt(sum(e * e for e in errors) / len(errors))
        final_drift = errors[-1]
        max_drift = max(errors)

    # 2D rigid alignment (Procrustes) removes frame heading/offset conventions.
    aligned_rmse = float("nan")
    heading_offset_deg = float("nan")
    if len(pairs) >= 2:
        ox = sum(o.x for o, _ in pairs) / len(pairs)
        oy = sum(o.y for o, _ in pairs) / len(pairs)
        tx = sum(t.x for _, t in pairs) / len(pairs)
        ty = sum(t.y for _, t in pairs) / len(pairs)
        a = sum((o.x - ox) * (t.x - tx) + (o.y - oy) * (t.y - ty) for o, t in pairs)
        b = sum((o.x - ox) * (t.y - ty) - (o.y - oy) * (t.x - tx) for o, t in pairs)
        theta = math.atan2(b, a)
        heading_offset_deg = math.degrees(theta)
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        residuals = []
        for o, t in pairs:
            dx, dy = o.x - ox, o.y - oy
            rx = cos_t * dx - sin_t * dy
            ry = sin_t * dx + cos_t * dy
            residuals.append(math.hypot(t.x - tx - rx, t.y - ty - ry))
        aligned_rmse = math.sqrt(sum(r * r for r in residuals) / len(residuals))

    def clean(value):
        return round(value, 4) if not math.isnan(value) else None

    return {
        "odom_samples": len(recorder.odom),
        "truth_samples": len(recorder.truth),
        "odom_rate_hz": round(odom_rate, 2),
        "odom_path_length_m": round(odom_length, 2),
        "truth_path_length_m": round(truth_length, 2),
        "position_rmse_m": clean(rmse),
        "final_drift_m": clean(final_drift),
        "max_drift_m": clean(max_drift),
        "aligned_rmse_m": clean(aligned_rmse),
        "heading_offset_deg": clean(heading_offset_deg),
    }


def main():
    parser = argparse.ArgumentParser(description="AeroSLAM LIO benchmark")
    parser.add_argument("--duration", type=float, default=45.0, help="seconds of flight per run")
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--scan-period", type=float, default=None,
                        help="per-point time spread for de-skew (0 disables it)")
    parser.add_argument("--outdir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "results"))
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    for run in range(args.runs):
        print("=== run %d/%d ===" % (run + 1, args.runs))
        launch_args = ["roslaunch", "aeroslam_bringup", "uav_livox_fastlio.launch",
                       "gui:=false", "headless:=true", "rviz:=false"]
        if args.scan_period is not None:
            launch_args.append("livox_scan_period:=%s" % args.scan_period)
        launch = subprocess.Popen(launch_args, stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        try:
            rospy.init_node("aeroslam_benchmark", anonymous=True, disable_signals=True)
            recorder = Recorder()
            if not wait_for_topics(timeout=90.0):
                print("topics did not appear; aborting run")
                continue
            enable_motors()
            time.sleep(2.0)
            flight = threading.Thread(target=fly_waypoints, args=(args.duration,))
            flight.start()
            flight.join()
            time.sleep(3.0)
            summary = summarise(recorder)
            summary["run"] = run + 1
            summary["duration_s"] = args.duration
            with open(os.path.join(args.outdir, "run_%02d.json" % (run + 1)), "w") as handle:
                json.dump(summary, handle, indent=2)
            for key in ("odom_rate_hz", "odom_path_length_m", "truth_path_length_m",
                        "position_rmse_m", "aligned_rmse_m", "heading_offset_deg",
                        "final_drift_m", "max_drift_m"):
                print("  %-22s %s" % (key, summary[key]))
        finally:
            os.killpg(os.getpgid(launch.pid), signal.SIGINT)
            try:
                launch.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(launch.pid), signal.SIGKILL)
            subprocess.call(["pkill", "-f", "gzserver"])
            time.sleep(5)

    return 0


if __name__ == "__main__":
    sys.exit(main())
