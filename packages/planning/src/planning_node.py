#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
from numpy.linalg import norm
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from std_msgs.msg import Float64, Int32MultiArray, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3
from numpy import ndarray
import yaml
import pprint

MOVEMENT = {
    0: 'left',
    1: 'straight',
    2: 'right',
}

SIGN = {
    'left-T-intersect': [0, 1],  # ?
    'right-T-intersect': [1, 2],  # -
    'T-intersection': [0, 2]  # +
}


def parse(map, db):
    new_map = {}
    for cross_num in map.keys():
        new_map[cross_num] = {}
        for index, cross_sign in enumerate(map[cross_num]['tag_id']):
            if cross_sign == '':
                continue
            tag_id = cross_sign
            cross_sign = db[cross_sign]['traffic_sign_type']
            move = SIGN[cross_sign]
            template_move = [[i, i] for i in move]
            move = [(i + index) % 4 for i in move]
            new_map[cross_num][tag_id] = {map[cross_num]['cross'][j]: template_move[i][1] for i, j in
                                          enumerate(move)}
    markers_db = {}
    for cross_i in new_map.keys():
        for id in new_map[cross_i]:
            markers_db[id] = cross_i
    return new_map, markers_db


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
        self.markers_conf = rospy.get_param('~markers_db')
        self.map_conf = rospy.get_param('~config')
        self.log('planning_node_init')
        with open(self.markers_conf) as stream:
            self.markers_db = yaml.safe_load(stream)
        with open(self.map_conf) as stream:
            self.map = yaml.safe_load(stream)
        self.state = self.map['state']
        self.map, self.markers_db = parse(self.map['map'], self.markers_db)
        self.log(self.map)
        self.log(self.markers_db)
        self.prev_state = 1
        self.trajectory = [2, 3]

        self.stop_tag_detection_pub = rospy.Publisher(
            '~stop_detection', Bool, queue_size=1
        )

        self.tag_sub = rospy.Subscriber(
            "~tags_id", Int32MultiArray, self.get_way, queue_size=1, buff_size="20MB"
        )

    def get_way(self, msg):
        stop_msg = Bool()
        stop_msg.data = True
        self.stop_tag_detection_pub.publish(stop_msg)
        data = msg.data
        cross = list(set([self.markers_db[i] for i in data]))[0]
        marker = self.state[cross]['prev'][self.prev_state]
        rotation = self.map[self.trajectory[0]][marker][self.trajectory[1]]
        self.log(f'trajectory == {rotation}')


if __name__ == "__main__":
    node = Planning()
    # spin forever
    rospy.spin()
