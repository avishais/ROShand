#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from marker_tracker.msg import ImageSpacePoseMsg
import math
import numpy as np


class print_state():

    gripper_pos = [0,0] 
    gripper_load = [0,0]
    base_pos = [0,0]
    base_theta = 0
    obj_pos = [0,0]
    R = []
    count = 1

    def __init__(self):

        rospy.Subscriber('/gripper/pos', Float32MultiArray, self.callbackGripperPos)
        rospy.Subscriber('/gripper/load', Float32MultiArray, self.callbackGripperLoad)
        rospy.Subscriber('/marker_tracker/image_space_pose_msg', ImageSpacePoseMsg, self.callbackMarkers)

        rospy.init_node('print_state', anonymous=True)
        rate = rospy.Rate(15) # 15hz
        while not rospy.is_shutdown():
            print(self.obj_pos, self.gripper_pos, self.gripper_load)
            rospy.spin()
            rate.sleep()

    def callbackGripperPos(self, msg):
        self.gripper_pos = msg.data

    def callbackGripperLoad(self, msg):
        self.gripper_load = msg.data

    def callbackMarkers(self, msg):
        try:
            self.base_pos = np.array([msg.posx[msg.ids.index(0)], msg.posy[msg.ids.index(0)]])
            bt = math.pi - msg.angles[msg.ids.index(0)]

            self.base_theta  = (self.count-1)/self.count * self.base_theta + bt/self.count
            if self.count > 1e7:
                self.count = 2
            else:
                self.count += 1
        except:
            pass
        try:
            self.obj_pos = np.array([msg.posx[msg.ids.index(5)], msg.posy[msg.ids.index(5)]])
            self.obj_pos = self.obj_pos - self.base_pos
            self.R = np.array([[math.cos(self.base_theta), -math.sin(self.base_theta)], [math.sin(self.base_theta), math.cos(self.base_theta)]])
            # print(self.R,self.base_theta)
            self.obj_pos = np.matmul(self.R, self.obj_pos.T)
        except:
            pass
 

if __name__ == '__main__':
    
    try:
        PS = print_state()
    except rospy.ROSInterruptException:
        pass