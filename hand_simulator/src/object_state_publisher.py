#!/usr/bin/python 

import rospy
import numpy as np 
from std_msgs.msg import Float64, Float32MultiArray
from gazebo_msgs.msg import ModelStates, LinkStates
import PyKDL



class object_state_publisher():

    obj_pos = np.array([0.,0.,0.])
    R_obj = None
    hand_pos = np.array([0.,0.,0.])
    R_hand = None
    msg = Float32MultiArray()
    

    def __init__(self):
        rospy.init_node('object_state_publisher', anonymous=True)

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.ObjCallback)
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.HandCallback)
        object_position_pub = rospy.Publisher('/hand/obj_pos', Float32MultiArray, queue_size=10)
        object_orientation_pub = rospy.Publisher('/hand/obj_orientation', Float32MultiArray, queue_size=10)

        rate = rospy.Rate(100)
        rate.sleep()
        while not rospy.is_shutdown():

            object_pos = self.obj_pos - self.hand_pos # Position relative to the hands base link
            obj_orientation = (self.R_hand*self.R_obj.Inverse()).GetEulerZYX() # Orientation relative to hand - I did not verify that this is correct

            self.msg.data = object_pos
            object_position_pub.publish(self.msg)
            self.msg.data = obj_orientation
            object_orientation_pub.publish(self.msg)

            rate.sleep()

    def ObjCallback(self, msg):

        idx = self.getNameOrder(msg.name, 'object')

        self.obj_pos = np.array([msg.pose[idx].position.x, msg.pose[idx].position.y, msg.pose[idx].position.z])
        self.R_obj = PyKDL.Rotation.Quaternion(msg.pose[idx].orientation.x, msg.pose[idx].orientation.y, msg.pose[idx].orientation.z, msg.pose[idx].orientation.w)
        

    def HandCallback(self, msg):

        idx = self.getNameOrder(msg.name, 'base_link')
        
        self.hand_pos = np.array([msg.pose[idx].position.x, msg.pose[idx].position.y, msg.pose[idx].position.z])
        self.R_hand = PyKDL.Rotation.Quaternion(msg.pose[idx].orientation.x, msg.pose[idx].orientation.y, msg.pose[idx].orientation.z, msg.pose[idx].orientation.w)
        
    def getNameOrder(self, names, str):

        idx = -1
        for i, name in enumerate(names):
            if name == str:
                idx = i
            if name[-9:] == str:
                idx = i

        return idx


if __name__ == '__main__':

    try:
        object_state_publisher()
    except rospy.ROSInterruptException:
        pass