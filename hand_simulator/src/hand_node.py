#!/usr/bin/python 

import rospy
import numpy as np 
from std_msgs.msg import Float64
from openhand.srv import MoveServos



class SimHandNode():

    def __init__(self, num_fingers = 2):
        rospy.init_node('SimHandNode', anonymous=True)

        #initialize service handlers:
        # rospy.Service('MoveServos',MoveServos,self.MoveServosProxy)

        pub_f1_jb1 = rospy.Publisher('/hand/base_to_finger_1_1_position_controller/command', Float64, queue_size=10)
        pub_f1_j12 = rospy.Publisher('/hand/finger_1_1_to_finger_1_2_position_controller/command', Float64, queue_size=10)
        pub_f1_j23 = rospy.Publisher('/hand/finger_1_2_to_finger_1_3_position_controller/command', Float64, queue_size=10)
        pub_f2_jb1 = rospy.Publisher('/hand/base_to_finger_2_1_position_controller/command', Float64, queue_size=10)
        pub_f2_j12 = rospy.Publisher('/hand/finger_2_1_to_finger_2_2_position_controller/command', Float64, queue_size=10)
        pub_f2_j23 = rospy.Publisher('/hand/finger_2_2_to_finger_2_3_position_controller/command', Float64, queue_size=10)
        pub_f3_jb1 = rospy.Publisher('/hand/base_to_finger_3_1_position_controller/command', Float64, queue_size=10)
        pub_f3_j12 = rospy.Publisher('/hand/finger_3_1_to_finger_3_2_position_controller/command', Float64, queue_size=10)
        pub_f3_j23 = rospy.Publisher('/hand/finger_3_2_to_finger_3_3_position_controller/command', Float64, queue_size=10)

        

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if num_fingers == 2:
                pub_f3_jb1.publish(0.)
                pub_f3_j12.publish(0.)
                pub_f3_j23.publish(0.)
                pub_f1_jb1.publish(np.pi/2)
                pub_f2_jb1.publish(-np.pi/2)

            # print(self.obj_pos)
            # rospy.spin()
            rate.sleep()


    # def MoveServosProxy(self,req):
    #     pos = req.pos


if __name__ == '__main__':

    try:
        SimHandNode(2)
    except rospy.ROSInterruptException:
        pass