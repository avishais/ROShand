#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Int16
from std_srvs.srv import SetBool, Empty, EmptyResponse
from rl_pkg.srv import TargetAngles, IsDropped, observation
from sim_transition_model.srv import action
import math
import numpy as np

import matlab.engine

global eng
global gp_obj

predict_mode = 1


class Spin_gp_hand_sim():

    gripper_load = np.array([0.0,0.0])
    obj_pos = np.array([0.0,0.0])
    R = []
    A = np.array([[0.2, 0.2], [-0.2, 0.2], [0.2, -0.2], [0.2, 0.2], [0, 0.4], [0.4, 0], [0, -0.4], [-0.4, 0]])
    count = 1
    # state_region = np.array([[-171.0700, -430.5800, -20.0000, -556.0000],[293.8600, -176.4200,  601.0000, 16.0000]]) # Real system with discrete actions
    state_region = np.array([[-86.9527,   50.4905,    1.4053,    2.7736],[90.0164,  134.0481,   61.1859,   58.2129]]) # Gazebo simulation
    state_dim = state_region.shape[1]
    state = np.zeros(state_dim)
    nn_radius = 0.9
    nn_num_bound = 10750
    dropped = False

    # KEY_W = 119
    # KEY_X = 120
    # KEY_S = 115
    # KEY_D = 100
    # KEY_A = 97

    mode_ = predict_mode

    save_std = False
    saved_stds = []

    def __init__(self):
        pub_obj_pos = rospy.Publisher('/hand/obj_pos', Float32MultiArray, queue_size=10)
        pub_load = rospy.Publisher('/gripper/load', Float32MultiArray, queue_size=10)

        rospy.Service('/ResetGripper', Empty, self.ResetGripper)
        rospy.Service('/MoveGripper', action, self.MoveGripper)
        rospy.Service('/IsObjDropped', IsDropped, self.CheckDropped)
        rospy.Service('/observation', observation, self.GetObservation)

        msg = Float32MultiArray()

        rospy.init_node('gp_hand_sim', anonymous=True)

        rate = rospy.Rate(15) # 15hz
        while not rospy.is_shutdown():
            msg.data = self.obj_pos
            pub_obj_pos.publish(msg)
            msg.data = self.gripper_load
            pub_load.publish(msg)
            # rospy.spin()
            rate.sleep()

    def ResetGripper(self, msg):
        # Perhaps loop until find a feasible state!!!
        for i in range(self.state_dim):
            self.state[i] = np.random.uniform(self.state_region[0][i], self.state_region[1][i])
        self.dropped = False

        self.obj_pos = self.state[:2]
        self.gripper_load = self.state[2:]

        print('[gp_hand_sim] Gripper reset.')

        return EmptyResponse()
        

    # Predicts the next step by calling the GP class - gets external state (for planner)
    def MoveGripper(self, req):
        
        matS = []
        matS = matlab.double([self.state[0], self.state[1], self.state[2], self.state[3]])
        matA = matlab.double(req.action)

        if self.getNN_(req.action) < self.nn_num_bound:
            print('[gp_hand_sim] Gripper fail. End of episode.')
            return {'success': False}

        print('[gp_hand_sim] Current state s: ' + str(matS) + ", action: " + str(matA))
        # raw_input()

        # [sp, sigma] = gp_obj.predict(matS, matA, nargout=2)
        sp, sigma = eng.predict(gp_obj, matS, matA, nargout=2)

        print('[gp_hand_sim] Predicted next state sp: ' + str(sp[0]) + ", sigma: " + str(sigma[0]))

        self.state = np.array(sp[0])
        self.obj_pos = self.state[:2]
        self.gripper_load = self.state[2:]

        return {'success': True}

    def GetObservation(self, msg):

        return {'state': self.state}

    # Counts the number of NN in the data for the current state-action
    def CheckDropped(self, req):

        return {'dropped': not self.dropped}

    def getNN_(self, action):
        matS = []
        matS = matlab.double([self.state[0], self.state[1], self.state[2], self.state[3]])
        matA = matlab.double(action)

        return eng.getNN(gp_obj, matS, matA, self.nn_radius, nargout=1)
        

if __name__ == '__main__':
    eng = matlab.engine.start_matlab()
    eng.addpath('/home/pracsys/catkin_ws/src/rutgers_collab/src/sim_transition_model/src')
    gp_obj = eng.gp_class(predict_mode, True)
    try:
        SP = Spin_gp_hand_sim()
    except rospy.ROSInterruptException:
        pass