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
import tf2_msgs.msg
import cv2
import time
import yaml

from djitellopy import Tello

from rclpy.node import Node
from std_msgs.msg import Empty, UInt8, UInt8, Bool, String, Int8
from sensor_msgs.msg import Image, Imu, BatteryState, Temperature, CameraInfo
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import ament_index_python
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Float64MultiArray 

drone_x=0
drone_y=0

goal_x=0
goal_y=0

start=0

thrs=0.5
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.declare_parameter('target_frame', 'telloCamera')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value
        self.subscription = self.create_subscription(String,'cmd_tello',self.listener_callback,1)
        self.subscription  # prevent unused variable warning

        #self.sub_drone_pose = self.create_subscription(tf2_msgs.msg.TFMessage,'tf',self.pose_callback,1)
        #self.subscription  # prevent unused variable warning
        # Declare parameters

        self.sub_drone_goto = self.create_subscription(String,'go_to',self.goto_callback,1)
        self.subscription  # prevent unused variable warning
        # Declare parameters

        Tello.RESPONSE_TIMEOUT = int(10.0)
        
        self.tello = Tello()
        self.tello.connect()
        self.pub_conclude = self.create_publisher(String, 'conclude', 10)
        self.pub_image_raw = self.create_publisher(Image, '/drone1/image_raw', 1)
        self.pub_camera_info = self.create_publisher(CameraInfo, '/drone1/camera_info', 1)
        self.pub_reached_goal = self.create_publisher(Int8, 'reached_goal', 1)
        self.pub_cmd_reset = self.create_publisher(String, 'cmd_tello', 1)

        self.pub_cmd_vel = self.create_publisher(Twist, '/drone1/cmd_vel', 1)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        

        self.start_video_capture()
        self.start_goto_pose()
        #self.start_tello_status()
        #self.start_tello_odom()
        #tello.takeoff()

        #tello.move_left(100)
        #tello.rotate_counter_clockwise(90)
        #tello.move_forward(100)
        #tello.land()
        self.timer = self.create_timer(0.5, self.on_timer)


    def on_timer(self):
        """
        Callback function.
        This function gets called at the specific time interval.
        """
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'world'
    
        trans = None
        
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                        'world',
                        'telloCamera',
                        now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        # Publish the 2D pose
        self.current_x = trans.transform.translation.x
        self.current_y = trans.transform.translation.y    
        print("I believe I am at x = %.3f and y=  %.3f "  % (trans.transform.translation.x, trans.transform.translation.y))
        roll, pitch, yaw = self.euler_from_quaternion(
        trans.transform.rotation.x,
        trans.transform.rotation.y,
        trans.transform.rotation.z,
        trans.transform.rotation.w)      
        self.current_yaw = yaw    
        msg = Float64MultiArray()
        msg.data = [self.current_x, self.current_y, self.current_yaw]   
        #self.publisher_2d_pose.publish(msg) 
   
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z # in radians
        



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

    #def pose_callback(self, msg):
        #drone_x=msg.transforms.transform.translation.x
        #drone_y=msg.transforms.transform.translation.y
        #print("I believe I am at x = %.2f and y=  %.2f "  % (drone_x, drone_y))

    def goto_callback(self, msg):
        start=1
        x = msg.data.split(";")
        self.get_logger().info('x goal: "%s"' % x[0])
        self.get_logger().info('y goal: "%s"' % x[1])
        goal_x=float(x[0].strip("\""))
        goal_y=float(x[1].strip("\""))
        
        

    # Start goal pose thread.
    def start_goto_pose(self, rate=1.0/2.0):
        def goto_pose_thread():
            msg=Twist()
            while start==1:
                if(abs(goal_x-drone_x>thrs)):
                    if(goal_x>drone_x):
                        msg.linear.x = 0.01
                    else:
                        msg.linear.x = -0.01
                elif(abs(goal_y-drone_y>thrs)):
                    if(goal_y>drone_y):
                        msg.linear.y = 0.01
                    else:
                        msg.linear.y = -0.01
                else:
                    msg.linear.x = 0
                    msg.linear.y = 0
                self.pub_cmd_vel.publish(msg) #Only for ROS-Gazebo Simulation

                time.sleep(rate)
                

        # We need to run the recorder in a seperate thread, otherwise blocking options would prevent frames from getting added to the video
        thread = threading.Thread(target=goto_pose_thread)
        thread.start()
        return thread
    
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
