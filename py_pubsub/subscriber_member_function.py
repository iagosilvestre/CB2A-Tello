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

from djitellopy import Tello
from std_msgs.msg import String
from sensor_msgs.msg import Image

import av
import math
import numpy as np
import threading
import time

tello = Tello()
x=0
y=0
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'cmd_tello',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.pub_image_raw = self.create_publisher(Image, 'image_raw', 10)
        self.pub_conclude = self.create_publisher(String, 'conclude', 10)
        
        self.frame_thread = threading.Thread(target=self.framegrabber_loop)
        self.frame_thread.start()
        #tello = Tello()

        #tello.connect()
        #tello.takeoff()

        #tello.move_left(100)
        #tello.rotate_counter_clockwise(90)
        #tello.move_forward(100)

        #tello.land()

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        x = msg.data.split(";")
        if x[0] == "takeoff":
        	print("takeoff")
        	tello.takeoff()
        if x[0] == "move_left":
        	tello.move_left(int(x[1]))
        elif x[0] == "move_right":
        	tello.move_right(int(x[1]))
        elif x[0] == "move_up":
        	tello.move_left(int(x[1]))
        elif x[0] == "move_down":
        	tello.move_left(int(x[1]))
        elif x[0] == "land":
        	tello.land()
        elif x[0] == "rotate_ccw":
        	tello.rotate_counter_clockwise(int(x[1]))
        elif x[0] == "rotate_cw":
        	tello.rotate_clockwise(int(x[1]))
        	
        	
        	
    def framegrabber_loop(self):
        tello.streamon()
        time.sleep(1.0)

        # Once connected, process frames till drone/stream closes
        while self.state != self.STATE_QUIT:
            try:
                frame = tello.get_frame_read()
                img = np.array(frame.to_image())
                img_msg = self.bridge.cv2_to_imgmsg(img, 'rgb8')
                img_msg.header.frame_id = rclpy.get_namespace()
                self.pub_image_raw.publish(img_msg)
            except BaseException as err:
                self.get_logger().info('Decoding error' )  
    

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
