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


class IntersectionController(DTROS):
    def __init__(self):
        super(IntersectionController, self).__init__(
            node_name="intersection_controller_node", node_type=NodeType.PERCEPTION
        )
        """
        - будет подписан на топик ошибки
        - здесь будет мб pid регулятор
        - нода отправлят v,w или vl, vr
        """
        self._error = rospy.Subscriber(
            "~error", Float64, self.cb_err, queue_size=1, buff_size="10MB"
        )  # TODO: remap в launch файле

        # publisher to wheels -- add

    def cb_err(self, error):
        pass


if __name__ == "__main__":
    node = IntersectionController()
    # spin forever
    rospy.spin()
