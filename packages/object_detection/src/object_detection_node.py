#!/usr/bin/env python3
from __future__ import annotations

import rospy
import torch
from PIL import Image 
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from sensor_msgs.msg import CompressedImage


COUNTER_FREQUENCY = 5

class ObjectDetectionNode(DTROS):
    def __init__(self) -> None:
        counter = 0
        super(ObjectDetectionNode, self).__init__(
            node_name="object_detection_node", node_type=NodeType.PERCEPTION
        )

        self.img_sub = rospy.Subscriber(
            "~image", CompressedImage, self.cb_image, queue_size=1, buff_size="20MB"
        )

        
        self.detected_objs = rospy.Publisher(
            "~objects_detected", str, queue_size=1   , dt_topic_type=TopicType.DRIVER
        )

    def cb_image(self, msg) -> None:
        def sort_func(tensor: torch.Tensor) -> float:
            x1, y1, x2, y2, _, _ = tensor
            return (x2 - x1) * (y2 - y1)
        
        if (counter := counter + 1) == COUNTER_FREQUENCY:
            counter = 0
            img = Image.fromarray(msg.data, 'RGB')
            response = list(self._model(img))
            response.sort(key=sort_func)
            # self.detected_objs.publish(response)

            self.log("Received image")

    



if __name__ == "__main__":
    node = ObjectDetectionNode()
    # spin forever
    rospy.spin()
