#!/usr/bin/python 

import rospy
import numpy as np 
from std_msgs.msg import Float64
from hand_simulator.srv import MoveServos
from std_msgs.msg import Float64MultiArray, Float32MultiArray



class SimHandNode():

    act_torque = np.array([0.,0.,0.,0.]) # left_proximal, left_lateral, right_proximal, right_lateral
    act_angles = None # actuator angles of all three fingers
    fingers_angles = np.array([0.,0.,0.,0.]) # left_proximal, left_lateral, right_proximal, right_lateral
    ref_angles = np.array([0.,0.,0.,0.,0.,0.]) # spring reference angle left_proximal, left_lateral, right_proximal, right_lateral    

    max_f = 300. # max tendon force
    h1 = 1; h2 = 0.5 # # Tendon force distribution on the finger
    Q = np.array([[max_f, 0., 0.],[0., max_f, 0.],[0., 0., max_f]])
    R = np.array([[h1,0.,0.],[h2,0.,0.],[0.,h1,0.],[0.,h2,0.],[0.,0.,h1],[0.,0.,h2]])

    k1 = 5
    k2 = 10
    K = np.diag([k1,k2,k1,k2,k1,k2]) # Springs coefficients
    

    def __init__(self, num_fingers = 2):
        rospy.init_node('SimHandNode', anonymous=True)

        self.num_fingers = num_fingers
        self.act_angles = np.zeros(num_fingers) # Normalized actuators angles [0,1]
        self.Q = self.Q[:num_fingers,:num_fingers]
        self.R = self.R[:2*num_fingers,:num_fingers]
        self.K = self.K[:2*num_fingers,:2*num_fingers]
        self.ref_angles = self.ref_angles[:2*num_fingers].reshape(2*num_fingers,1)

        #initialize service handlers:
        rospy.Service('MoveServos',MoveServos,self.MoveServosProxy)

        rospy.Subscriber('/hand/my_joint_states', Float32MultiArray, self.JointStatesCallback)

        self.pub_f1_jb1 = rospy.Publisher('/hand/base_to_finger_1_1_position_controller/command', Float64, queue_size=10)
        self.pub_f1_j12 = rospy.Publisher('/hand/finger_1_1_to_finger_1_2_position_controller/command', Float64, queue_size=10)
        self.pub_f1_j23 = rospy.Publisher('/hand/finger_1_2_to_finger_1_3_position_controller/command', Float64, queue_size=10)
        self.pub_f2_jb1 = rospy.Publisher('/hand/base_to_finger_2_1_position_controller/command', Float64, queue_size=10)
        self.pub_f2_j12 = rospy.Publisher('/hand/finger_2_1_to_finger_2_2_position_controller/command', Float64, queue_size=10)
        self.pub_f2_j23 = rospy.Publisher('/hand/finger_2_2_to_finger_2_3_position_controller/command', Float64, queue_size=10)
        self.pub_f3_jb2 = rospy.Publisher('/hand/base_to_finger_3_2_position_controller/command', Float64, queue_size=10)
        self.pub_f3_j23 = rospy.Publisher('/hand/finger_3_2_to_finger_3_3_position_controller/command', Float64, queue_size=10)

        self.gripper_angles_pub = rospy.Publisher('/gripper/pos', Float32MultiArray, queue_size=10)
        self.gripper_load_pub = rospy.Publisher('/gripper/load', Float32MultiArray, queue_size=10)
        msg = Float32MultiArray()

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.num_fingers == 2:
                self.pub_f3_jb2.publish(0.)
                self.pub_f3_j23.publish(0.)
                self.pub_f1_jb1.publish(np.pi/2)
                self.pub_f2_jb1.publish(-np.pi/2)

            tendon_forces = self.Q.dot( self.act_angles.reshape(self.num_fingers,1) )
            self.act_torque = self.R.dot( tendon_forces ) - self.K.dot( self.fingers_angles - self.ref_angles )

            self.pub_f1_j12.publish(self.act_torque[0])
            self.pub_f1_j23.publish(self.act_torque[1])
            self.pub_f2_j12.publish(self.act_torque[2])
            self.pub_f2_j23.publish(self.act_torque[3])
            if self.num_fingers > 2:
                self.pub_f3_jb2.publish(self.act_torque[4])
                self.pub_f3_j23.publish(self.act_torque[5])
            
            msg.data = self.act_angles
            self.gripper_angles_pub.publish(msg)
            msg.data = tendon_forces
            self.gripper_load_pub.publish(msg)

            rate.sleep()

    def JointStatesCallback(self, msg):
        angles = np.array(msg.data)

        self.fingers_angles = angles[[1,2,4,5,6,7]].reshape(6,1)
        self.fingers_angles = self.fingers_angles[:2*self.num_fingers]

    def MoveServosProxy(self,req):
        self.act_angles = np.array(req.pos[:self.num_fingers])

        # Enforce normalized actuator angles
        for i in range(self.num_fingers):
            self.act_angles[i] = min(self.act_angles[i],1)
            self.act_angles[i] = max(self.act_angles[i],0)

        return 0


if __name__ == '__main__':

    try:
        SimHandNode(2)
    except rospy.ROSInterruptException:
        pass