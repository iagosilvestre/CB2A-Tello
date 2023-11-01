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


tello = Tello()

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'cmd_tello',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
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
        if x[0] == "move_right":
        	tello.move_right(int(x[1]))
        if x[0] == "move_up":
        	tello.move_left(int(x[1]))
        if x[0] == "move_down":
        	tello.move_left(int(x[1]))


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
