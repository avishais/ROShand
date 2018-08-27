#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from marker_tracker.msg import ImageSpacePoseMsg
from gp_predict.srv import StateAction, getNN, ActionChoice, StateAction2State
import math
import numpy as np

from predict_GPy import predict
# from predict_pyGPs import predict

class Spin_predict(predict):

    gripper_pos = [0,0] 
    gripper_load = [0,0]
    base_pos = [0,0]
    base_theta = 0
    obj_pos = [0,0]
    R = []
    A = np.array([[0.06, 0.06], [-0.06, 0.06], [0.06, -0.06], [0.06, 0.06]])
    count = 1

    KEY_W = 119
    KEY_X = 120
    KEY_S = 115
    KEY_D = 100
    KEY_A = 97

    mode_ = 1

    def __init__(self):
        predict.__init__(self, self.mode_)

        rospy.Subscriber('/gripper/pos', Float32MultiArray, self.callbackGripperPos)
        rospy.Subscriber('/gripper/load', Float32MultiArray, self.callbackGripperLoad)
        rospy.Subscriber('/marker_tracker/image_space_pose_msg', ImageSpacePoseMsg, self.callbackMarkers)
        s1 = rospy.Service('predict', StateAction, self.callbackPredictService)
        s2 = rospy.Service('NumNN', getNN, self.callbackNumNNService)
        s3 = rospy.Service('ChooseAction', ActionChoice, self.callbackChooseAction)
        s4 = rospy.Service('predictWithState', StateAction2State, self.callbackPredictServiceWithState)

        rospy.init_node('predict', anonymous=True)
        rate = rospy.Rate(15) # 15hz
        while not rospy.is_shutdown():
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

    def callbackChooseAction(self, req):
        cur_set_point = req.set_point

        if self.mode_==1:
            s = self.obj_pos
        if self.mode_==4:
            s = np.concatenate((self.obj_pos, self.gripper_pos), axis=0)
        if self.mode_==5:
            s = np.concatenate((self.obj_pos, self.gripper_load), axis=0)
        if self.mode_==8:
            s = np.concatenate((self.obj_pos, self.gripper_pos, self.gripper_load), axis=0)

        print('----------------------------')
        print('Current position: ', np.matmul(self.R.T, self.obj_pos)+self.base_pos, ', waypoint: ', cur_set_point, '--:')
        print('Current load: ', self.gripper_load)
        print('Current gripper pos: ' + str(self.gripper_pos))
        print('base' + str(self.base_pos) + ' ' + str(self.base_theta))

        d_max = 1e9
        a_max = self.A[0,:]
        for i in range(self.A.shape[0]):
            a = self.A[i,:]
            sa = np.concatenate((s, a), axis=0)

            sa = sa.reshape((1,self.state_action_dim))
            sa = self.normalize(sa, 1)
            s_next = self.propagate(sa).reshape(1, self.state_dim)
            s_next = self.denormalize(s_next, 2)

            # Take only image coordinates
            s_next = s_next[0,:2]

            # Reproject to image plane
            s_next = np.matmul(self.R.T, s_next.T)
            s_next += self.base_pos

            d = np.linalg.norm(s_next-cur_set_point)
            print("Action " +str(self.A[i,:]) + " predicts next state " + str(s_next) + " with distance " + str(d))
            if d < d_max:
                d_max = d
                a_max = self.A[i,:]
                s_next_s = s_next

        a = a_max
        if a[0] < 0 and a[1] < 0: # Down
            K = self.KEY_W
            # return {'key': self.KEY_W}

        if a[0] > 0 and a[1] > 0: # Up
            K = self.KEY_X
            # return {'key': self.KEY_X}
        
        if a[0] < 0 and a[1] > 0: # Left
            K = self.KEY_D
            # return {'key': self.KEY_A}

        if a[0] > 0 and a[1] < 0: # right
            K = self.KEY_A
            # return {'key': self.KEY_D}

        if a[0] == 0 and a[1] == 0:
            K = self.KEY_S
            # return {'key': self.KEY_S}

        print("*** predicted next state: " + str(s_next_s) + " with action: " + str(a) + "(" + str(K) + ")" + " planned to have distance " + str(np.linalg.norm(s_next_s-cur_set_point)))
        return {'key': K}

    # Predicts the next step by calling the GP class
    def callbackPredictService(self, req):
        a = np.array(req.action)

        if self.mode_==1:
            sa = np.concatenate((self.obj_pos, a), axis=0)
        if self.mode_==4:
            sa = np.concatenate((self.obj_pos, self.gripper_pos, a), axis=0)
        if self.mode_==5:
            sa = np.concatenate((self.obj_pos, self.gripper_load, a), axis=0)
        if self.mode_==8:
            sa = np.concatenate((self.obj_pos, self.gripper_pos, self.gripper_load, a), axis=0)

        sa = sa.reshape((1,self.state_action_dim))

        sa = self.normalize(sa, 1)

        s_next = self.propagate(sa).reshape(1, self.state_dim)

        s_next = self.denormalize(s_next, 2)

        # Take only image coordinates
        s_next = s_next[0,:2]

        # Reproject to image plane
        s_next = np.matmul(self.R.T, s_next.T)
        s_next += self.base_pos

        # print('s_next in image space: ' + str(s_next))
        
        return {'next_state': s_next}

    # Predicts the next step by calling the GP class - gets external state (for planner)
    def callbackPredictServiceWithState(self, req):
        s = np.array(req.state)
        a = np.array(req.action)

        print('-----------------------------------------------------------------------------------------------')
        print('state: ' + str(s) + ', action: ' + str(a))

        sa = np.concatenate((s, a), axis=0)

        sa = sa.reshape((1,self.state_action_dim))

        sa = self.normalize(sa, 1)

        s_next = self.propagate(sa).reshape(1, self.state_dim)

        s_next = self.denormalize(s_next, 2)

        print('Predicted next state: ' + str(s_next))
     
        return {'next_state': s_next[0]}

    # Counts the number of NN in the data for the current state-action
    def callbackNumNNService(self, req):
        a = np.array(req.action)

        if self.mode_==1:
            sa = np.concatenate((self.obj_pos, a), axis=0)
        if self.mode_==4:
            sa = np.concatenate((self.obj_pos, self.gripper_pos, a), axis=0)
        if self.mode_==5:
            sa = np.concatenate((self.obj_pos, self.gripper_load, a), axis=0)
        if self.mode_==8:
            sa = np.concatenate((self.obj_pos, self.gripper_pos, self.gripper_load, a), axis=0)

        sa = sa.reshape((1,self.state_action_dim))
        sa = self.normalize(sa, 1)

        k = self.countNN(sa)

        rospy.loginfo('[gp_predict] Found ' + str(k) + ' nearest-neighbors for action ' + str(a) + ' and current configuration.')
       
        return {'num': k}
   

if __name__ == '__main__':
    
    try:
        SP = Spin_predict()
    except rospy.ROSInterruptException:
        pass