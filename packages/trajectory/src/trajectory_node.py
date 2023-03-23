#!/usr/bin/env python3

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

        self._odm_error_pub = rospy.Publisher(
            "~error", Float64, queue_size=1
        )

        self.log('trajectory_node_init')

        self.current_point = None
        self.vector = Vector3(0, 0, 0)

        self.goal_point = Point()
        self.goal_vector = Vector3(0, 0, 0)

    def cb_odom(self, msg: Odometry):
        # self.last_point_x = msg.pose.pose.position.x
        # if not self.start_point:
        #     self.start_point = self.last_point_x
        # if msg.pose.pose.position.x != self.last_point_x:
        #     self.log(msg.pose.pose.position.x)
        #     self.last_point_x = msg.pose.pose.position.x
        # self.log(msg.pose.pose)
        # self.log(msg.pose.pose.position.x)
        if not self.current_point:
            self.update_cur_point(msg.pose.pose.position)
            return
        self.update_vector(self.vector, msg.pose.pose.position)
        self.update_cur_point(msg.pose.pose.position)

    def update_current_point(self, new_point):
        self.current_point.x = new_point.x
        self.current_point.y = new_point.y
        self.current_point.z = new_point.z

    def update_vector(self, vector: Vector3, new_point: Point):
        vector.x = new_point.x - self.current_point.x
        vector.y = new_point.y - self.current_point.y
        vector.z = new_point.z - self.current_point.z

    # not checked
    def error(self):
        val = 1 - np.dot(self.vector, self.goal_vector) / (norm(self.vector) * norm(self.goal_vector))
        self._odm_error_pub.publish(val)

    # not checked
    def dist(self, p1: Point, p2, Point) -> float | ndarray:
        return np.linalg.norm(p1 - p2)


if __name__ == "__main__":
    node = Trajectory()
    # spin forever
    rospy.spin()
