#!/usr/bin/env python3
from __future__ import annotations

import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from sensor_msgs.msg import CompressedImage

class ObjectDetectionNode(DTROS):
    def __init__(self):
        super(ObjectDetectionNode, self).__init__(
            node_name="object_detection_node", node_type=NodeType.PERCEPTION
        )

        self.img_sub = rospy.Subscriber(
            "~image", CompressedImage, self.cb_image, queue_size=1, buff_size="20MB"
        )

    def cb_image(self, msg):
        self.log("Received image")


if __name__ == "__main__":
    node = ObjectDetectionNode()
    # spin forever
    rospy.spin()
