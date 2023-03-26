#!/usr/bin/env python3
from __future__ import annotations

import cv2
import math
import rospy
import numpy as np
from numpy.linalg import norm
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
from std_msgs.msg import Float64, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3
from numpy import ndarray

P, I, D = 1, 0, 0

DEFAULT_LINEAR_SPEED = 0.2


class PIDController(DTROS):
    def __init__(self):
        super(PIDController, self).__init__(
            node_name="pid_controller_node", node_type=NodeType.PERCEPTION
        )
        self.base_line = 0.1
        self.r = 0.0318
        self.limit = 1
        self._goal_sub = rospy.Subscriber(
            "~error", Float64, self.cb_err, queue_size=1, buff_size="10MB"
        )

        self._wheel_pub = rospy.Publisher(
            "~car_cmd", WheelsCmdStamped, queue_size=1
        )
        self.integral = 0
        self.derivative = 0
        self.prev_error = 0

        self.log('pid_controller_node_init')

    def cb_err(self, msg: Float64):
        error = float(msg.data)  # NO CHECKED
        if math.isnan(error):
            error = 0
        self.integral += error
        self.derivative = error - self.prev_error
        self.prev_error = error

        # отправка сообщения

        v = DEFAULT_LINEAR_SPEED
        omega = P * error + I * self.integral + D * self.derivative
        v1 = ((v + 0.5 * omega * self.base_line) / self.r) / 22
        v2 = ((v - 0.5 * omega * self.base_line) / self.r) / 22
        u_r_limited = max(min(v1, self.limit), -self.limit)
        u_l_limited = max(min(v2, self.limit), -self.limit)
        # TODO: FIX MEEEEEEEEEEEEEE
        msg = WheelsCmdStamped(vel_left=0, vel_right=0)

        #self.log(msg)
        #self._wheel_pub.publish(msg)


if __name__ == "__main__":
    node = PIDController()
    # spin forever
    rospy.spin()
