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


class Planning(DTROS):
    def __init__(self):
        super(Planning, self).__init__(
            node_name="planning_node", node_type=NodeType.PERCEPTION
        )
        """
        - Получить id поворота
        - Создать путь
        - Обновлять цель (внутри маршрута)
        """
        self._odm_sub = rospy.Subscriber(
            "~odom", Odometry, self.cb_odom, queue_size=1, buff_size="10MB"
        )

        self._odm_error_pub = rospy.Publisher(
            "~error", Float64, queue_size=1
        )

        self.log('planning_node_init')


if __name__ == "__main__":
    node = Planning()
    # spin forever
    rospy.spin()
