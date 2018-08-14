#!/usr/bin/env python
import rospy
from common_msgs_gl.srv import SendBool
from marker_tracker.msg import ImageSpacePoseMsg
from std_msgs.msg import UInt32
import math
import numpy as np
from matplotlib import pyplot as plt
from gp_predict.srv import StateAction


class track():

    counter = 0
    enable = False
    obj_pos = np.array([0,0])
    path = np.array([0,0])
    ready = False
    A = np.array([[0.06, 0.06], [-0.06, 0.06], [0.06, -0.06]])#, [0.06, 0.06]])
    freq = 15

    KEY_W = 119
    KEY_X = 120
    KEY_S = 115
    KEY_D = 100
    KEY_A = 97

    def callbackTrackEnable(self, req):
        self.enable = req.data


    def tracked_path(self, obj_pos):
        self.path = obj_pos.reshape(1,2)
        for i in range(1,20):
            self.path = np.append(self.path, np.array([obj_pos[0]+i*1, obj_pos[1]]).reshape(1,2), axis=0) 
        for i in range(1,15):
            self.path = np.append(self.path, np.array([self.path[-1,0]+0.8, self.path[-1,1]+0.8]).reshape(1,2), axis=0) 
        plt.plot(self.path[:,0], self.path[:,1],'.b')
        plt.draw()
        self.ready = True

    def callbackMarkers(self, msg):
        try:
            self.obj_pos = np.array([msg.posx[msg.ids.index(5)], msg.posy[msg.ids.index(5)]])
            if self.counter % 2 == 0:
                plt.plot(self.obj_pos[0], self.obj_pos[1],'.r')
                plt.axis("equal")
                plt.axis([200, 750, 90, 350])
                plt.draw()
                plt.pause(0.00000000001)
            self.counter += 1

            # print(self.obj_pos, self.counter)

            if self.counter == 5:
                self.tracked_path(self.obj_pos)
        except:
            pass


    def predict_client(self, a):
        rospy.wait_for_service('predict')
        try:
            predict = rospy.ServiceProxy('predict', StateAction)
            res = predict(a)
            return np.array(res.next_state)
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)

    def choose_action(self, cur_set_point):
        d_max = 1e9
        for i in range(self.A.shape[0]):
            s_next = self.predict_client(self.A[i,:])
            d = np.linalg.norm(s_next-cur_set_point)
            if d < d_max:
                d_max = d
                a = self.A[i,:]

        if a[0] < 0 and a[1] < 0: # Down
            return self.KEY_W

        if a[0] > 0 and a[1] > 0: # Up
            return self.KEY_X
        
        if a[0] < 0 and a[1] > 0: # Left
            return self.KEY_A

        if a[0] > 0 and a[1] < 0: # right
            return self.KEY_D

        if a[0] == 0 and a[1] == 0:
            return self.KEY_S


    def __init__(self):
        iter = 0
        
        sr = rospy.Service('/trackEnable', SendBool, self.callbackTrackEnable)
        rospy.Subscriber('/marker_tracker/image_space_pose_msg', ImageSpacePoseMsg, self.callbackMarkers)
        pub = rospy.Publisher('/keyboard_input', UInt32, queue_size=10)

        rospy.init_node('track', anonymous=True)
        rate = rospy.Rate(self.freq) # 15hz
        while not rospy.is_shutdown():
            if self.enable and self.ready:
                print(iter)
                cur_set_point = self.path[iter,:]
                a = self.choose_action(cur_set_point)
                pub.publish(a)
                rospy.sleep(0*1/self.freq+1)
                pub.publish(self.KEY_S)
                if iter+1==self.path.shape[0]:
                    self.enable = False
                iter += 1

            # plt.show()
            # rospy.spin()
            rate.sleep()

if __name__ == '__main__':
    
    try:
        track()
    except rospy.ROSInterruptException:
        pass