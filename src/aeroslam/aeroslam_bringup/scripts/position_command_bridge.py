#!/usr/bin/env python3
"""Bridge EGO-Planner output to the hector quadrotor control interface.

Subscribes : quadrotor_msgs/PositionCommand  (topic: /planning/pos_cmd)
Publishes  : geometry_msgs/PoseStamped        (topic: command/pose)

The hector position controller consumes PoseStamped commands on command/pose
(the same interface used by hector_quadrotor_actions/pose_action). The bridge
republishes the planner trajectory reference at a fixed rate so the controller
always sees a fresh setpoint.

Parameters (private):
  ~input_topic   (default /planning/pos_cmd)
  ~output_topic  (default command/pose)
  ~rate          (default 50.0 Hz)
  ~z_offset      (default 0.0 m)
"""

import math

import rospy
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand


class PositionCommandBridge(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/planning/pos_cmd")
        self.output_topic = rospy.get_param("~output_topic", "command/pose")
        self.rate_hz = rospy.get_param("~rate", 50.0)
        self.z_offset = rospy.get_param("~z_offset", 0.0)

        self.last_cmd = None
        self.yaw = 0.0

        self.publisher = rospy.Publisher(self.output_topic, PoseStamped, queue_size=1)
        self.subscriber = rospy.Subscriber(self.input_topic, PositionCommand, self.command_callback)
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.timer_callback)

        rospy.loginfo("position_command_bridge: %s -> %s (%.1f Hz)",
                      self.input_topic, self.output_topic, self.rate_hz)

    def command_callback(self, msg):
        self.last_cmd = msg
        if abs(msg.velocity.x) > 0.05 or abs(msg.velocity.y) > 0.05:
            self.yaw = math.atan2(msg.velocity.y, msg.velocity.x)
        elif abs(msg.yaw_dot) > 1e-3:
            self.yaw = msg.yaw

    def timer_callback(self, _event):
        if self.last_cmd is None:
            return

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "world"
        pose.pose.position.x = self.last_cmd.position.x
        pose.pose.position.y = self.last_cmd.position.y
        pose.pose.position.z = self.last_cmd.position.z + self.z_offset
        pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.publisher.publish(pose)


def main():
    rospy.init_node("position_command_bridge")
    PositionCommandBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
