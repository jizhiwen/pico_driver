# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import numpy as np
from multiprocessing import shared_memory
import threading
import json
from pico_driver.image_client import ImageClient
from pico_driver.vuer_server import VuerServer

class VuerServerWrap(Node):
    def __init__(self):
        super().__init__('pico_state_publisher')
        self.publisher_ = self.create_publisher(String, '/pico_driver/pose', 10)

        img_shape = (480, 640, 3)
        img_shm = shared_memory.SharedMemory(create=True, size=np.prod(img_shape) * np.uint8().itemsize)
        img_array = np.ndarray(img_shape, dtype=np.uint8, buffer=img_shm.buf)
        img_client = ImageClient(tv_img_shape = img_shape, tv_img_shm_name = img_shm.name,
                             server_address="127.0.0.1", port=6666)
        image_receive_thread = threading.Thread(target=img_client.receive_process, daemon=True)
        image_receive_thread.start()
        self.vuer_server = VuerServer(False, img_shape, img_shm.name,
                                      cert_file="cert.pem", 
                                      key_file="key.pem", 
                                      host="0.0.0.0", 
                                      port=8012)
        
        server_thread = threading.Thread(target=self.polling_routine)
        server_thread.start()

    def polling_routine(self):
        while True:
            value = self.vuer_server.on_update()

            msg = String()
            msg.data = json.dumps(value)
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    # minimal_publisher = MinimalPublisher()
    vuer_server = VuerServerWrap()

    rclpy.spin(vuer_server)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vuer_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
