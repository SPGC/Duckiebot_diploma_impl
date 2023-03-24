#!/usr/bin/env python3
from __future__ import annotations

import cv2
import rospy
import numpy as np
from numpy.linalg import norm
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3
from numpy import ndarray


class Trajectory(DTROS):
    def __init__(self):
        super(Trajectory, self).__init__(
            node_name="trajectory_node", node_type=NodeType.PERCEPTION
        )
        self._odm_sub = rospy.Subscriber(
            "~odom", Odometry, self.cb_odom, queue_size=1, buff_size="10MB"
        )

        self._goal_sub = rospy.Subscriber(
            "~goal", Point, self.cb_update_goal, queue_size=1, buff_size="10MB"
        )

        self._odm_error_pub = rospy.Publisher(
            "~error", Float64, queue_size=1
        )
        self._dist_goal = rospy.Publisher(
            "~dist", Float64, queue_size=1
        )

        self._state_pub = rospy.Publisher(
            "~state_vector", Vector3, queue_size=1
        )

        self._pos_pub = rospy.Publisher(
            "~state_pos", Point, queue_size=1
        )
        self.log('trajectory_node_init')

        self.current_point = None
        self.state = Vector3(0, 0, 0)

        self.goal_point = Point()
        self.goal_state = Vector3(0, 0, 0)

    def cb_odom(self, msg: Odometry):
        # self.last_point_x = msg.pose.pose.position.x
        # if not self.start_point:
        #     self.start_point = self.last_point_x
        # if msg.pose.pose.position.x != self.last_point_x:
        #     self.log(msg.pose.pose.position.x)
        #     self.last_point_x = msg.pose.pose.position.x
        # self.log(msg.pose.pose)
        # self.log(msg.pose.pose.position.x)
        data_pos = msg.pose.pose.position
        if not self.current_point:
            self.current_point = Point()
            self.update_current_point(data_pos)
            return
        self.log(f"{self.is_moving(data_pos)} :: {data_pos.x} :: {self.current_point.x}")
        if self.is_moving(data_pos):
            self.update_state(data_pos)
            self.update_current_point(data_pos)
            self.update_goal_state_vector()
        self._pos_pub.publish(self.current_point)

    def cb_update_goal(self, goal: Point):
        self.goal_point = goal

    def update_current_point(self, new_point):
        self.current_point.x = new_point.x
        self.current_point.y = new_point.y
        self.current_point.z = new_point.z

    def update_state(self, new_point: Point):
        self.state.x = new_point.x - self.current_point.x
        self.state.y = new_point.y - self.current_point.y
        self.state.z = new_point.z - self.current_point.z
        self.log(self.state)
        self._state_pub.publish(self.state)

    def update_goal_state_vector(self):
        self.goal_state.x = self.goal_point.x - self.current_point.x
        self.goal_state.y = self.goal_point.y - self.current_point.y
        self.goal_state.z = self.goal_point.z - self.current_point.z
        self.update_error_and_dist()

    def update_error_and_dist(self):
        self._odm_error_pub.publish(self.error())
        self._dist_goal.publish(self.dist())

    # not checked
    def error(self):
        v1 = np.array((self.state.x, self.state.y))
        v2 = np.array((self.goal_state.x, self.goal_state.y))
        val = 1 - np.dot(v1, v2) / (norm(v1) * norm(v2))
        return val

    # not checked
    def dist(self) -> float | ndarray:
        v1 = np.array((self.goal_point.x, self.goal_point.y))
        v2 = np.array((self.current_point.x, self.current_point.y))
        return np.linalg.norm(v1 - v2)

    def is_moving(self, p: Point) -> Point:
        return self.current_point.x != p.x and self.current_point.y != p.y


if __name__ == "__main__":
    node = Trajectory()
    # spin forever
    rospy.spin()
