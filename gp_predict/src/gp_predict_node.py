#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from predict_GPy import predict
from marker_tracker.msg import ImageSpacePoseMsg
from gp_predict.srv import StateAction
import math
import numpy as np

class Spin_predict(predict):

    gripper_pos = [0,0] 
    gripper_load = [0,0]
    base_pos = [0,0]
    base_theta = 0
    obj_pos = [0,0]
    flag = 0
    R = []

    mode_ = 1
    # GP = []

    def __init__(self):
        predict.__init__(self, self.mode_)
        # GP = predict(self.mode)

        pub = rospy.Publisher('prediction', Float64MultiArray, queue_size=10)
        pub_status = rospy.Publisher('prediction_status', Float64MultiArray, queue_size=10)
        rospy.Subscriber('/gripper/pos', Float32MultiArray, self.callbackGripperPos)
        rospy.Subscriber('/gripper/load', Float32MultiArray, self.callbackGripperLoad)
        rospy.Subscriber('/marker_tracker/image_space_pose_msg', ImageSpacePoseMsg, self.callbackMarkers)
        s = rospy.Service('predict', StateAction, self.callbackPredictService)

        rospy.init_node('predict', anonymous=True)
        rate = rospy.Rate(1) # 10hz
        array = [0.,0.]
        a = Float64MultiArray(data=array)
        # a.data = [0, 0]
        while not rospy.is_shutdown():
            # rospy.loginfo(a)
            # pub.publish(a)
            # array[0] += 1.1
            # array[1] += 2.4
            # a = Float64MultiArray(data=array)
            rospy.spin()
            rate.sleep()

    def is_empty(self, any_structure):
        if any_structure:
            return False
        else:
            return True

    def callbackGripperPos(self, msg):
        self.gripper_pos = msg.data
        

    def callbackGripperLoad(self, msg):
        self.gripper_load = msg.data

    def callbackMarkers(self, msg):
        try:
            self.base_pos = np.array([msg.posx[msg.ids.index(0)], msg.posy[msg.ids.index(0)]])
            self.base_theta = math.pi - msg.angles[msg.ids.index(0)]
        except:
            pass
        try:
            self.obj_pos = np.array([msg.posx[msg.ids.index(5)], msg.posy[msg.ids.index(5)]])
            self.obj_pos = self.obj_pos - self.base_pos
            self.R = np.array([[math.cos(self.base_theta), -math.sin(self.base_theta)], [math.sin(self.base_theta), math.cos(self.base_theta)]])
            self.obj_pos = np.matmul(R, self.obj_pos.T)
        except:
            pass

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

        print(sa)
        sa = sa.reshape((1,self.state_action_dim))
        sa = self.normalize(sa, 1)
        s_next = self.propagate(sa).reshape(1, self.state_dim)
        s_next = self.denormalize(s_next)
        print(type(s_next), s_next)

        s_next = np.matmul(self.R.T, s_next[0].T)

        s_next += self.base_pos
        print(s_next.shape, s_next.T)
        
        return {'next_state': s_next}

   

if __name__ == '__main__':
    
    try:
        SP = Spin_predict()
    except rospy.ROSInterruptException:
        pass