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
from visualization_msgs.msg import Marker
from numpy import ndarray
from scipy import interpolate

TILE_SIZE = 0.616
EPS = 0.03 # 3 cm
KIND = "kind"
POINTS = "points"
FREQUENCY = "freq"

PATH_SETTINGS = {
    FREQUENCY: 4,
    0: {  # left
        KIND: 2,
        POINTS: [(3 / 4, 0.0), (1 / 2, 1 / 2), (0, 3 / 4)]
    },
    1: {  # straight
        KIND: 1,
        POINTS: [(3 / 4, 0.0), (3 / 4, 1)]
    },
    2: {  # right
        KIND: 2,
        POINTS: [(3 / 4, 0.0), (7 / 8, 3 / 16), (1, 1 / 4)]
    }
}

COLOURS = {
    0: 'y',  # 'YELLOW',
    1: 'c',  # 'BLUE',
    2: 'g',  # 'GREEN'
}


def _get_interpolate_path(turn_id):  # 0 1 2
    path_config = PATH_SETTINGS[turn_id]
    kind = path_config[KIND]
    points = path_config[POINTS]
    x = np.array([TILE_SIZE * kx for kx, _ in points])
    y = np.array([TILE_SIZE * ky for _, ky in points])
    xp = np.linspace(x[0], x[-1], PATH_SETTINGS[FREQUENCY])
    if turn_id == 1:
        raise NotImplemented
    fp = interpolate.interp1d(x, y, kind=kind)
    return [np.array((x, fp(x))) for x in xp]


def get_cos(v1, v2):
    return np.dot(v1, v2) / (norm(v1) * norm(v2))


class PlanningTrajectory(DTROS):
    def __init__(self):
        super(PlanningTrajectory, self).__init__(
            node_name="planning_trajectory_node", node_type=NodeType.PERCEPTION
        )
        # вектор, который определяет движение робота
        self.is_was_first_point = True
        self._state_sub = rospy.Subscriber(
            "~state", Vector3, self.cb_state, queue_size=1, buff_size="10MB"
        )

        # чтобы знать откуда прокладывать путь
        self._state_pos_sub = rospy.Subscriber(
            "~state_pos", Point, self.cb_pos, queue_size=1, buff_size="10MB"
        )
        # расстояние робота до goal точки
        # self._dist_sub = rospy.Subscriber(
        #     "~dist_by_goal", Float64, self.cb_goal_by_dist, queue_size=1, buff_size="10MB"
        # )

        # тип поворота: left, straight, right
        self._turn_type_sub = rospy.Subscriber(
            "~turn_type", Int8, self.cb_turn_type, queue_size=1, buff_size="10MB"
        )

        # Создаем следующую goal точку
        self._goal_pub = rospy.Publisher(
            "~goal", Point, queue_size=1
        )

        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)

        # Данные из real
        self.real_pos = np.array([0, 0])  # current_point
        self.real_state = np.array([])  # вектор куда сейчас смотрит робот
        self.turn_type = None

        # Данные которые используются для расчетов в системе координаты перекреста
        self.way = deque()  # cписок точек, куда поедет робот
        self.prev_inter_point = Point()
        self.inter_point = Point()
        self.goal: ndarray = np.array([])  # точка, до которой должен добраться дакибот
        self.goal_inter_vector = None
        self.current_inter_vector = None

    def cb_state(self, state: Vector3):
        self.real_state = np.array([state.x, state.y])
        self.log("CB_STATE")

    # def cb_goal_by_dist(self, dist: Float64):
    #     self.log(dist)
    #     # get next goal from deque(self.way)
    #     self.log("CB_GOAL BY DIST")

    def cb_turn_type(self, turn_type: Int8):
        turn_type = int(turn_type.data)
        if self.turn_type == turn_type:
            self.log("The same turn type. No update")
            return

        self.turn_type = turn_type
        # generate way
        self.way = deque(_get_interpolate_path(turn_id=turn_type))
        # self.prev_inter_point = self.way.pop()
        self.inter_point = self.way.popleft()

        # self.goal_inter_vector = np.array([
        #     self.inter_point[0] - self.prev_inter_point[0],
        #     self.inter_point[1] - self.prev_inter_point[1]
        # ])
        self.current_inter_vector = np.array([0, 1])  # в самом начале перекрестка бот смотрит прямо
        self.log(f"TURN TYPE == {turn_type}")
        self.is_was_first_point = False

    def cb_pos(self, point: Point):
        prev_real_pos = self.real_pos
        self.real_pos = np.array([point.x, point.y])
        if self.is_destination() or not self.is_was_first_point:
            self.is_was_first_point = True
            self.prev_inter_point = self.inter_point
            self.inter_point = self.way.popleft()

            self.goal_inter_vector = np.array([
                self.inter_point[0] - self.prev_inter_point[0],
                self.inter_point[1] - self.prev_inter_point[1]
            ])

            cosin = get_cos(self.goal_inter_vector, self.current_inter_vector)
            next_step = self._get_next_step_vector(cosin,
                                                   self.real_state,
                                                   self.goal_inter_vector)
            self.goal = prev_real_pos + next_step
            self.current_inter_vector = self.goal_inter_vector
        # if True:
        #    val = 0
        #    self._goal_pub.publish(val)
        if len(self.goal) == 2:
            val = Point(self.goal[0], self.goal[1], 0)
            self._goal_pub.publish(val)

    def is_destination(self) -> bool:
        # проверка добрались ли до goal точки
        if len(self.goal) == 2:
            v1 = np.array((self.goal[0], self.goal[1]))
            v2 = np.array((self.real_pos[0], self.real_pos[1]))
            self.log(f"Distance to next point: {np.linalg.norm(v1 - v2)} :: v1:{v1} :: v2:{v2}")
            return np.linalg.norm(v1 - v2) < EPS
        return False

    def _get_next_step_vector(self, cosin, v1, v2):
        norm_orig = v1 / norm(v1)
        sin_alpha = np.sqrt(1 - cosin ** 2)
        x = norm_orig[0] * cosin - norm_orig[1] * sin_alpha
        y = norm_orig[1] * cosin + norm_orig[0] * sin_alpha

        res = np.array((x, y)) * (norm(v1) / norm(norm_orig)) * (norm(v2) / norm(v1))
        return res


if __name__ == "__main__":
    node = PlanningTrajectory()
    # spin forever
    rospy.spin()
