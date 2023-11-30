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

import pprint
import math
import rclpy
import threading
import numpy
import time
import av
import tf2_ros
import cv2
import time
import yaml

from djitellopy import Tello

from rclpy.node import Node
from std_msgs.msg import Empty, UInt8, UInt8, Bool, String, Int8
from sensor_msgs.msg import Image, Imu, BatteryState, Temperature, CameraInfo
from geometry_msgs.msg import Twist, TransformStamped, PoseStampedMsg
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import ament_index_python

drone_x=0
drone_y=0
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String,'cmd_tello',self.listener_callback,1)
        self.subscription  # prevent unused variable warning

        self.sub_drone_pose = self.create_subscription(PoseStampedMsg,'pose_orb',self.pose_callback,1)
        self.subscription  # prevent unused variable warning
        # Declare parameters

        self.sub_drone_pose = self.create_subscription(String,'go_to',self.goto_callback,1)
        self.subscription  # prevent unused variable warning
        # Declare parameters

        Tello.RESPONSE_TIMEOUT = int(10.0)
        
        self.tello = Tello()
        self.tello.connect()
        self.pub_conclude = self.create_publisher(String, 'conclude', 10)
        self.pub_image_raw = self.create_publisher(Image, 'drone1/image_raw', 1)
        self.pub_camera_info = self.create_publisher(CameraInfo, 'drone1/camera_info', 1)
        self.pub_reached_goal = self.create_publisher(Int8, 'reached_goal', 1)
        self.pub_cmd_reset = self.create_publisher(String, 'cmd_tello', 1)
        
        
        

        self.start_video_capture()
        #self.start_tello_status()
        #self.start_tello_odom()
        #tello.takeoff()

        #tello.move_left(100)
        #tello.rotate_counter_clockwise(90)
        #tello.move_forward(100)

        #tello.land()

    def listener_callback(self, msg):
        x = msg.data.split(";")
        self.get_logger().info('I heard: "%s"' % x[0])
        if x[0] == "\"takeoff":
            self.tello.takeoff()
        elif x[0] == "\"move_left":
            self.tello.move_left(int(x[1].strip("\"")))
        elif x[0] == "\"move_right":
            self.tello.move_right(int(x[1].strip("\"")))
        elif x[0] == "\"move_forward":
            self.tello.move_forward(int(x[1].strip("\"")))
        elif x[0] == "\"move_back":
            self.tello.move_back(int(x[1].strip("\"")))
        elif x[0] == "\"land":
            self.tello.land()
        elif x[0] == "\"rotate_ccw":
            self.tello.rotate_counter_clockwise(int(x[1].strip("\"")))
        elif x[0] == "\"rotate_cw":
            self.tello.rotate_clockwise(int(x[1].strip("\"")))

    def pose_callback(self, msg):
        drone_x=msg.pose.position.x
        drone_y=msg.pose.position.y

    def goto_callback(self, msg):
        x = msg.data.split(";")
        self.get_logger().info('I heard: "%s"' % x[0])
        goal_x=int(x[0].strip("\""))
        goal_y=int(x[1].strip("\""))
        
        

    # Start video capture thread.
    def start_video_capture(self, rate=1.0/15.0):
        # Enable tello stream
        self.tello.streamon()

        # OpenCV bridge
        self.bridge = CvBridge()

        def video_capture_thread():
            frame_read = self.tello.get_frame_read()

            while True:
                # Get frame from drone
                frame = frame_read.frame

                # Publish opencv frame using CV bridge
                msg = self.bridge.cv2_to_imgmsg(numpy.array(frame), 'rgb8')
                msg.header.frame_id = 'drone'
                self.pub_image_raw.publish(msg)

                time.sleep(rate)
                

        # We need to run the recorder in a seperate thread, otherwise blocking options would prevent frames from getting added to the video
        thread = threading.Thread(target=video_capture_thread)
        thread.start()
        return thread 	
      
    

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
