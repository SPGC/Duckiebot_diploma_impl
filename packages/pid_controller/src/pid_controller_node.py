#!/usr/bin/env python3
from __future__ import annotations

import cv2
import rospy
import numpy as np
from numpy.linalg import norm
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Float64, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3
from numpy import ndarray

P, I, D = .1, 0, 0

DEFAULT_LINEAR_SPEED = 0.5


class PIDController(DTROS):
    def __init__(self):
        super(PIDController, self).__init__(
            node_name="pid_controller_node", node_type=NodeType.PERCEPTION
        )

        self._goal_sub = rospy.Subscriber(
            "~error", Float64, self.cb_err, queue_size=1, buff_size="10MB"
        )

        self._wheel_pub = rospy.Publisher(
            "~car_cmd", Twist2DStamped, queue_size=1
        )
        self.integral = 0
        self.derivative = 0
        self.prev_error = 0

        self.log('pid_controller_node_init')

    def cb_err(self, msg: Float64):
        error = msg.data  # NO CHECKED
        self.integral += error
        self.derivative = error - self.prev_error
        self.prev_error = error

        # отправка сообщения
        msg = Twist2DStamped()
        msg.v = DEFAULT_LINEAR_SPEED
        msg.omega = P * error + I * self.integral + D * self.derivative
        self._wheel_pub.publish(msg)


if __name__ == "__main__":
    node = PIDController()
    # spin forever
    rospy.spin()
