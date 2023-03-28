#!/usr/bin/env python3
from __future__ import annotations

import socket
import rospy
import pickle

# import torch
# from PIL import Image 

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from sensor_msgs.msg import CompressedImage

COUNTER_FREQUENCY = 5

IP = '420.13.37.69'
PORT = 1337

clientSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
clientSocket.connect((IP, PORT))

class ObjectDetectionNode(DTROS):
    def __init__(self) -> None:
        self.counter = 0
        super(ObjectDetectionNode, self).__init__(
            node_name="object_detection_node", node_type=NodeType.PERCEPTION
        )

        # self._model = torch.hub.load('ultralytics/yolov5', 'custom', path = 'model.pt')
        # self._model.load_state_dict(torch.load('model.pt')['model'].state_dict())

        self._img_sub = rospy.Subscriber(
            "~image", CompressedImage, self.cb_image, queue_size=1, buff_size="20MB"
        )

        self._detected_objs = rospy.Publisher(
            "~objects_detected", str, queue_size=1, dt_topic_type=TopicType.DRIVER
        )

    def cb_image(self, msg) -> None:
        # def sort_func(tensor: torch.Tensor) -> float:
        #     x1, y1, x2, y2, _, _ = tensor
        #     return (x2 - x1) * (y2 - y1)
        
        
        # dataFromServer = clientSocket.recv(1024)
        # print(dataFromServer.decode())
        
        self.counter += 1
        if self.counter == COUNTER_FREQUENCY:
            send_package = pickle.dumps(msg.data)
            clientSocket.send(send_package)
            self.counter = 0
            decoded_data = clientSocket.recv(4_000_000).decode()
            # Not sure what kind of objects will be sent (if it is an array of integers, it will be awful)
            self.log(str(len(decoded_data)))
            return
            # self.counter 
            requests.post()
            img = Image.fromarray(msg.data, 'RGB')
            response = list(self._model(img))
            response.sort(reverse=True, key=sort_func)
            # self._detected_objs.publish(response)
            self.log("Received image")
    

if __name__ == "__main__":
    node = ObjectDetectionNode()
    # spin forever
    rospy.spin()
