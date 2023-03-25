#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
from numpy.linalg import norm
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from std_msgs.msg import Float64, Int32MultiArray, Bool, Int32, String, Int8
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


def parse(map, db, checker=False):
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
    state = {}
    for cross_num in map.keys():
        state[cross_num] = {'prev': {}}
        for i, tag_id in enumerate(map[cross_num]['tag_id']):
            if tag_id == '' or map[cross_num]['cross'][(3 + i) % 4] == '':
                continue
            state[cross_num]['prev'][map[cross_num]['cross'][(3 + i) % 4]] = tag_id

    return new_map, markers_db, state


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
        self.zone1 = rospy.get_param('~zone1')
        self.zone2 = rospy.get_param('~zone2')
        self.zone3 = rospy.get_param('~zone3')
        self.full = rospy.get_param('~full')
        self.autolab = rospy.get_param('~autolab')

        with open(self.markers_conf) as stream:
            self.markers_db_yaml = yaml.safe_load(stream)
        self.log('planning_node_init')
        self.map_name = None
        self.prev_state = None
        self.trajectory = None
        self.state = None
        self.trajectory_sub = rospy.Subscriber(
            '~trajectory', Int32MultiArray, self.update_trajectory, queue_size=1
        )
        self.begin_state_sub = rospy.Subscriber(
            '~begin_state', Int32, self.update_begin_state, queue_size=1
        )

        self.map_sub = rospy.Subscriber(
            '~map_state', String, self.update_map, queue_size=1
        )

        self.tag_sub = rospy.Subscriber(
            "~tags_id", Int32MultiArray, self.get_way, queue_size=1, buff_size="20MB"
        )

        self.end_trajectory_sub = rospy.Subscriber(
            "~end_trajectory", Bool, self.update_trajectory_after_cross, queue_size=1, buff_size="20MB"
        )

        self.stop_tag_detection_pub = rospy.Publisher(
            '~stop_detection', Bool, queue_size=1
        )


        self.rotation_pub = rospy.Publisher(
            '~rotation', Int8, queue_size=1
        )

    def update_trajectory_after_cross(self, msg):
        self.log('update vals')
        self.log(f'prev before, {self.prev_state}, traj == {self.trajectory}')
        self.prev_state = self.trajectory[0]
        del self.trajectory[0]
        self.log(f'prev after, {self.prev_state}, traj == {self.trajectory}')
        if len(self.trajectory) == 1:
            self.log('next stop is destination')

    def update_state(self):
        map_conf = None
        if self.map_name == 'zone1':
            map_conf = self.zone1
        elif self.map_name == 'zone2':
            map_conf = self.zone2
        elif self.map_name == 'zone3':
            map_conf = self.zone3
        elif self.map_name == 'full':
            map_conf = self.full
        elif self.map_name == 'autolab':
            map_conf = self.autolab
        with open(map_conf) as stream:
            self.map = yaml.safe_load(stream)
        self.state = self.map
        self.map, self.markers_db, self.state = parse(self.map, self.markers_db_yaml)
        if self.map_name == 'zone1' or self.map_name == 'full':
            self.state[1]['prev'][2] = 152
            self.state[2]['prev'][1] = 143
        self.log(self.map)
        self.log(self.markers_db)

    def update_trajectory(self, msg):
        self.trajectory = list(msg.data)
        self.log(f'traj init == {self.trajectory}')

    def update_begin_state(self, msg):
        self.prev_state = msg.data
        self.log(f'prev_state init == {self.prev_state}')

    def update_map(self, msg):
        self.map_name = msg.data
        self.log(f'tmp_map init == {self.map_name}')
        self.update_state()
        self.log('map ready to run')

    def get_way(self, msg):
        self.log('in get_way planning method')
        if self.state is None or self.map is None or self.prev_state is None:
            self.log('in process to init values')
        else:
            stop_msg = Bool()
            stop_msg.data = True
            self.stop_tag_detection_pub.publish(stop_msg)
            data = msg.data
            self.log(f'markers in planning node {data}')
            cross = list(set([self.markers_db[i] for i in data]))
            self.log(f'current cross {cross}')
            cross = cross[0]
            if cross == self.trajectory[0] and len(self.trajectory) == 1:
                self.log('destination')
            else:
                self.log(f'state = {self.state}, prev == {self.prev_state}, cross == {cross}')
                marker = self.state[cross]['prev'][self.prev_state]
                rotation = self.map[self.trajectory[0] if self.trajectory[0] != 100 and self.trajectory[0] != 1000 and
                                                          self.trajectory[0] != 20 and self.trajectory[0] != 200 else
                int(str(self.trajectory[0])[0])][marker][self.trajectory[1]]
                self.rotation_pub.publish(Int8(data=rotation))
                self.log(f'trajectory == {rotation}')


if __name__ == "__main__":
    node = Planning()
    # spin forever
    rospy.spin()
