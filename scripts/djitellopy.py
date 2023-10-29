#!/usr/bin/env python
import rospy
import rospkg
import rosnode
import math
import time

from std_srvs.srv import Trigger as Trigger
from std_srvs.srv import SetBool as SetBool
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest, ApplyBodyWrench
from geometry_msgs.msg import Point, Wrench, Vector3

from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Float64, Int8,String,Bool

class djitellopy:
    def __init__(self):
        # Initialize node
        rospy.init_node('djitellopy')

        #critical things
        self.reaction_times = []
        self.perception_times = []
        
        self.perception_altitude = []
        self.reaction_altitude = []

        # Create publisher
        #self.percept_pub=rospy.Publisher ('/failure_uav1', Int8, queue_size=1)
        #self.block_pub=rospy.Publisher ('/block', Int8, queue_size=1)
        #self.unblock_pub=rospy.Publisher ('/unblock', Int8, queue_size=1)
        #self.motor1 = rospy.ServiceProxy('/uav1/control_manager/motors', SetBool)
        #self.tracker = rospy.ServiceProxy('/uav1/control_manager/switch_tracker', mrsString)
        #self.arm1 = rospy.ServiceProxy('/uav1/mavros/cmd/arming', CommandBool)
        #self.setmode = rospy.ServiceProxy('/uav1/mavros/set_mode', SetMode)
        #self.gotoalt1 = rospy.ServiceProxy('/uav1/control_manager/goto_altitude', Vec1)
        
        
        
        #self.file = open("ariacReactionTimes.log", "w")
        #self.file2 = open("altitudes.log", "w")
        self.count = 0
        self.min = 10
        self.isFinished = False
        self.startTime = time.time()

        # Create subscriber
        rospy.Subscriber('/agent_detected_failure_uav1', String, self.reaction)
        rospy.Subscriber('finish', Bool, self.callback2)
        rospy.Subscriber('/uav1/odometry/altitude', Float64Stamped, self.altitude)

    def run(self):
        i=0
        rate = rospy.Rate(0.05) # One failure every ~ 20 seconds 
        msg = Int8()
        msg2= Int8()
        perceptAlt = Float64Stamped()
        while not (rospy.is_shutdown() or self.isFinished or i>20):
            #self.gotoalt1(10)
            msg.data = i
            msg2.data = i
            self.block_pub.publish(msg2)
            time.sleep(2)
            if(i>=1):
                self.reaction_altitude.append(self.min) # Saves perception timestamp
                #rospy.loginfo("Received msg: %f", self.min)
            self.percept_pub.publish(msg) # Publishes failure and turns off UAV motor
            self.motor1(0) 
            self.perception_times.append(time.perf_counter()) # Saves perception timestamp
            perceptAlt = rospy.wait_for_message('/uav1/odometry/altitude', Float64Stamped)
            self.perception_altitude.append(perceptAlt.value) # Saves perception timestamp
            #rospy.loginfo("Received msg: %f", perceptAlt.value)
            i=i+1
            self.min = 10
            rate.sleep()
        self.recordTimes() # Before closing the rospy node, stores the collected timestamps
        self.recordAlt()
        rospy.signal_shutdown('Node is shutting down.')

    def recordAlt(self):
        cnt=0
        altLen=len(self.reaction_altitude)
        while(cnt<altLen):
            self.file2.write(f"{self.perception_altitude[cnt]}\t{self.reaction_altitude[cnt]}\n")
            cnt=cnt+1
        self.file2.close()
    
    def recordTimes(self):
        if len(self.perception_times) == len(self.reaction_times):
            for pTime, rTime in zip(self.perception_times, self.reaction_times):
                elapsed_time = (rTime - pTime) * 1000 
                self.file.write(f"{pTime}\t{rTime}\t{elapsed_time}\n")
        else:
            pIter = iter(self.perception_times)
            next(pIter)
            for rTime in self.reaction_times:
                pTime = next(pIter)
                elapsed_time = (rTime - pTime) * 1000 
                self.file.write(f"{pTime}, {rTime}, {elapsed_time}\n")
        total_time = time.time()-self.startTime        
        if not self.file.closed:
            self.file.write(f"Sent msgs: {len(self.perception_times)}\n")
            self.file.write(f"Received msgs: {len(self.reaction_times)}\n")
            self.file.write(f"Elapsed_time(s): {total_time}\n")
            self.file.close()

    def altitude(self, message):
        if(message.value<self.min):
            self.min=message.value
    
    
    def reaction(self, message):
        self.tracker('MpcTracker')
        self.arm1(1)
        self.motor1(1)
        self.reaction_times.append(time.perf_counter())
        msg= Int8()
        msg.data = self.count
        time.sleep(2)
        self.unblock_pub.publish(msg)
        self.count=self.count+1
            
    def callback2(self, message):
        # Print received message
        self.isFinished = True
        rospy.loginfo("FINISHED")

if __name__ == '__main__':
    node = djitellopy()
    time.sleep(1)
    node.run()
