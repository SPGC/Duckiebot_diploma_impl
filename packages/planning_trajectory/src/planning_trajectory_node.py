#!/usr/bin/env python3
from __future__ import annotations

from collections import deque

import cv2
import rospy
import numpy as np
from numpy.linalg import norm
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from std_msgs.msg import Float64, Int32MultiArray, Int16, Int8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3
from numpy import ndarray


class PlanningTrajectory(DTROS):
    def __init__(self):
        super(PlanningTrajectory, self).__init__(
            node_name="planning_trajectory_node", node_type=NodeType.PERCEPTION
        )
        # вектор, который определяет движение робота
        self._state_sub = rospy.Subscriber(
            "~state", Vector3, self.cb_state, queue_size=1, buff_size="10MB"
        )

        # чтобы знать откуда прокладывать путь
        self._state_pos_sub = rospy.Subscriber(
            "~state_pos", Point, self.cb_pos, queue_size=1, buff_size="10MB"
        )
        # расстояние робота до goal точки
        self._dist_sub = rospy.Subscriber(
            "~dist_by_goal", Float64, self.cb_goal_by_dist, queue_size=1, buff_size="10MB"
        )

        # тип поворота: left, straight, right
        self._turn_type_sub = rospy.Subscriber(
            "~turn_type", Int8, self.cb_turn_type, queue_size=1, buff_size="10MB"
        )

        # Создаем следующую goal точку
        self._goal_pub = rospy.Publisher(
            "~goal", Point, queue_size=1
        )

        self.pos = Point()
        self.state = Vector3()
        self.way = deque()
        self.turn_type = None

    def cb_state(self, state: Vector3):
        self.state = state
        self.log("CB_STATE")

    def cb_goal_by_dist(self, dist: Float64):
        self.log(dist)
        # get next goal from deque(self.way)
        self.log("CB_GOAL BY DIST")

    def cb_turn_type(self, turn_type: Int8):
        self.turn_type = turn_type
        # generate way
        self.log("TURN TYPE")

    def cb_pos(self, point: Point):
        self.pos = point
        self.log("CB_POS")
        val = self.pos
        val.x += 0.2
        self._goal_pub.publish(val)


if __name__ == "__main__":
    node = PlanningTrajectory()
    # spin forever
    rospy.spin()
