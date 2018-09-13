#!/usr/bin/env python

import rospy
from common_msgs_gl.srv import SendBool
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from marker_tracker.msg import ImageSpacePoseMsg
from std_msgs.msg import UInt32
import math
import numpy as np
import random

class auto_move():

    enable = False

    gripper_load = [0,0]
    base_pos = [0,0]
    base_theta = 0
    obj_pos = [0,0]
    R = []
    count = 1
    last_action = 0

    KEYS = np.array([113, 119, 101, 97, 100, 120, 99, 122])
    OP = np.array([[113, 99], [101, 122], [97, 100], [119, 120]])

    high_load = False

    pub = []

    def __init__(self):


        sr = rospy.Service('/auto_move/enable', SendBool, self.callbackEnable)
        rospy.Subscriber('/gripper/load', Float32MultiArray, self.callbackGripperLoad)
        rospy.Subscriber('/marker_tracker/image_space_pose_msg', ImageSpacePoseMsg, self.callbackMarkers)
        self.pub = rospy.Publisher('/keyboard_input', UInt32, queue_size=10)

        rospy.init_node('auto_move', anonymous=True)

        rospy.loginfo("[auto_move] Node initiated. Call /auto_move/enable service to enable motion")
        
        self.apply_actions()


    def random_action(self):
        return random.randint(0, 7)

    def apply_actions(self):
        rate = rospy.Rate(15) # 15hz


        while not rospy.is_shutdown():

            # print(self.gripper_load)

            if self.enable:

                while 1:
                    action = self.random_action()
                    if action != self.last_action:
                        break

                length = random.randint(10, 500)
                for i in range(length):
                    if np.abs(self.gripper_load[0]) > 350 or np.abs(self.gripper_load[1]) > 350:
                        self.high_load = True
                        break
                    self.pub.publish(self.KEYS[action])

                    # rospy.spin()
                    rate.sleep()

                if self.high_load:
                    h = np.where(OP == action)
                    hc = 1 - h[1][0]
                    new_action = OP[h[0][0]][hc]
                    for i in range(20):
                        self.pub.publish(self.KEYS[new_action])

                self.last_action = action
                    

            # rospy.spin()
            rate.sleep()

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
            self.obj_pos = np.matmul(self.R, self.obj_pos.T)
        except:
            pass

    def callbackEnable(self, req):
        self.enable = req.data


if __name__ == '__main__':
    
    try:
        SP = auto_move()
    except rospy.ROSInterruptException:
        pass