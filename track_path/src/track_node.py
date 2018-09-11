#!/usr/bin/env python
import rospy
from common_msgs_gl.srv import SendBool
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from marker_tracker.msg import ImageSpacePoseMsg
from std_msgs.msg import UInt32
import math
import numpy as np
from matplotlib import pyplot as plt
from gp_predict.srv import StateAction, ActionChoice


class track():

    counter = 0
    enable = False
    obj_pos = np.array([0,0])
    base_pos = np.array([0,0])
    gripper_load = np.array([0,0])
    base_theta = 0
    path = np.array([0,0])
    ready = False
    freq = 15
    #                  Q            W          E         A          D         Z        X       C
    A = np.array([[0.0, -0.4], [-0.2,-0.2], [-0.4,0],[0.2,-0.2],[-0.2,0.2],[0.4,0],[0.2,0.2],[0,0.4]])
    KEY_Q = 113
    KEY_W = 119
    KEY_E = 101
    KEY_A = 97
    KEY_S = 115
    KEY_D = 100
    KEY_Z = 122
    KEY_X = 120
    KEY_C = 99

    def callbackTrackEnable(self, req):
        self.enable = req.data


    def tracked_path(self, obj_pos):
        # Line
        # self.path = np.empty([5,2])
        # for i in range(1,self.path.shape[0]+1):
        #     self.path[i-1,:] = np.array([obj_pos[0]+i*12, obj_pos[1]+i*5]).reshape(1,2)
        # # for i in range(2,7):
        # #     self.path = np.append(self.path, np.array([self.path[-1,0]+10, self.path[-1,1]]).reshape(1,2), axis=0) 

        # square
        # self.path = np.empty([2,2])
        # st = 8
        # for i in range(1,self.path.shape[0]+1):
        #     self.path[i-1,:] = np.array([obj_pos[0]-i*st, obj_pos[1]]).reshape(1,2)
        # for i in range(1,4):
        #     self.path = np.append(self.path, np.array([self.path[-1,0], self.path[-1,1]+st]).reshape(1,2), axis=0) 
        # for i in range(1,4):
        #     self.path = np.append(self.path, np.array([self.path[-1,0]+st, self.path[-1,1]]).reshape(1,2), axis=0)
        # for i in range(1,4):
        #     self.path = np.append(self.path, np.array([self.path[-1,0], self.path[-1,1]-st]).reshape(1,2), axis=0)  
        # for i in range(1,4):
        #     self.path = np.append(self.path, np.array([self.path[-1,0]-st, self.path[-1,1]]).reshape(1,2), axis=0) 

        # Rectangle
        m = 4
        if m==1:
            self.path = np.empty([8,2])
            st = 8
            for i in range(1,self.path.shape[0]+1):
                self.path[i-1,:] = np.array([obj_pos[0]-i*st, obj_pos[1]]).reshape(1,2)
            for i in range(1,3):
                self.path = np.append(self.path, np.array([self.path[-1,0], self.path[-1,1]+st-3]).reshape(1,2), axis=0) 
            for i in range(1,9):
                self.path = np.append(self.path, np.array([self.path[-1,0]+st, self.path[-1,1]]).reshape(1,2), axis=0)
            for i in range(1,3):
                self.path = np.append(self.path, np.array([self.path[-1,0], self.path[-1,1]-(st-3)]).reshape(1,2), axis=0)  
        if m==2:
            self.path = np.empty([2,2])
            st = 8
            for i in range(1,self.path.shape[0]+1):
                self.path[i-1,:] = np.array([obj_pos[0], obj_pos[1]-i*st]).reshape(1,2)
            for i in range(1,8):
                self.path = np.append(self.path, np.array([self.path[-1,0]+st, self.path[-1,1]]).reshape(1,2), axis=0)
            for i in range(1,3):
                self.path = np.append(self.path, np.array([self.path[-1,0], self.path[-1,1]+st]).reshape(1,2), axis=0)
            for i in range(1,8):
                self.path = np.append(self.path, np.array([self.path[-1,0]-st, self.path[-1,1]]).reshape(1,2), axis=0) 
        if m==3:
            self.path = np.empty([8,2])
            st = 8
            for i in range(1,self.path.shape[0]+1):
                self.path[i-1,:] = np.array([obj_pos[0]+i*st, obj_pos[1]]).reshape(1,2)
            for i in range(1,3):
                self.path = np.append(self.path, np.array([self.path[-1,0], self.path[-1,1]+st-3]).reshape(1,2), axis=0) 
            for i in range(1,9):
                self.path = np.append(self.path, np.array([self.path[-1,0]-st, self.path[-1,1]]).reshape(1,2), axis=0)
            for i in range(1,3):
                self.path = np.append(self.path, np.array([self.path[-1,0], self.path[-1,1]-(st-3)]).reshape(1,2), axis=0)  
        if m==4:
            self.path = np.empty([2,2])
            st = 8
            for i in range(1,self.path.shape[0]+1):
                self.path[i-1,:] = np.array([obj_pos[0], obj_pos[1]+i*st]).reshape(1,2)
            for i in range(1,8):
                self.path = np.append(self.path, np.array([self.path[-1,0]+st, self.path[-1,1]]).reshape(1,2), axis=0)
            for i in range(1,3):
                self.path = np.append(self.path, np.array([self.path[-1,0], self.path[-1,1]-st]).reshape(1,2), axis=0)
            for i in range(1,8):
                self.path = np.append(self.path, np.array([self.path[-1,0]-st, self.path[-1,1]]).reshape(1,2), axis=0) 
         
        # Circle/Ellipse
        m = 3
        if m == 1:
            r = 24.0
            c = np.array([obj_pos[0], obj_pos[1]-r])
            Th = np.linspace(-np.pi/2, 1.5*np.pi, num=36)
            i = 0
            self.path = np.empty((0,2))
            for th in Th:
                self.path = np.append(self.path, np.array([c[0]+r*np.cos(th), c[1]-0.15*r*np.sin(th)+0.85*r]).reshape(1,2), axis=0) 
            self.path = np.delete(self.path, 0, 0)
        if m == 2:
            r = 24.0
            c = np.array([obj_pos[0], obj_pos[1]-r])
            Th = np.linspace(1.5*np.pi, -np.pi/2, num=36)
            i = 0
            self.path = np.empty((0,2))
            for th in Th:
                self.path = np.append(self.path, np.array([c[0]+r*np.cos(th), c[1]-0.18*r*np.sin(th)+0.82*r]).reshape(1,2), axis=0) 
            self.path = np.delete(self.path, 0, 0)
        if m == 3:
            r = 24.0
            c = np.array([obj_pos[0], obj_pos[1]+r])
            Th = np.linspace(1.5*np.pi, -np.pi/2, num=36)
            i = 0
            self.path = np.empty((0,2))
            for th in Th:
                self.path = np.append(self.path, np.array([c[0]+r*np.cos(th), c[1]+0.15*r*np.sin(th)-0.85*r]).reshape(1,2), axis=0) 
            self.path = np.delete(self.path, 0, 0)
        if m == 4:
            r = 24.0
            c = np.array([obj_pos[0], obj_pos[1]+r])
            Th = np.linspace(-np.pi/2, 1.5*np.pi, num=36)
            i = 0
            self.path = np.empty((0,2))
            for th in Th:
                self.path = np.append(self.path, np.array([c[0]+r*np.cos(th), c[1]+0.18*r*np.sin(th)-0.82*r]).reshape(1,2), axis=0) 
            self.path = np.delete(self.path, 0, 0)


        plt.plot(self.path[:,0], -self.path[:,1],'.b')
        plt.draw()
        self.ready = True

    # def callbackGripperLoad(self, msg):
    #     self.gripper_load = msg.data

    def callbackMarkers(self, msg):
        try:
            self.obj_pos = np.array([msg.posx[msg.ids.index(5)], msg.posy[msg.ids.index(5)]])
            self.base_pos = np.array([msg.posx[msg.ids.index(0)], msg.posy[msg.ids.index(0)]])
            self.base_theta = math.pi - msg.angles[msg.ids.index(0)]
            if self.counter % 7 == 0:
                plt.plot(self.obj_pos[0], -self.obj_pos[1],'.r')
                plt.axis("equal")
                # plt.axis([200, 750, -350, -40])
                # plt.axis([200, 750, 40, 350])
                plt.title("Obj. position: " + str(self.obj_pos))
                plt.draw()
                plt.pause(0.00000000001)
            self.counter += 1

            # print(self.obj_pos, self.counter)

            if self.counter == 5:
                self.tracked_path(self.obj_pos)
        except:
            pass


    def choose_action_client(self, carrot_point):
        rospy.wait_for_service('ChooseAction')
        try:
            ChooseAction = rospy.ServiceProxy('ChooseAction', ActionChoice)
            res = ChooseAction(carrot_point)
            return res.key
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)

    def __init__(self):
        iter = 0
        carrot_point = np.array([-1, -1])
        
        sr = rospy.Service('/trackEnable', SendBool, self.callbackTrackEnable)
        # rospy.Subscriber('/gripper/load', Float32MultiArray, self.callbackGripperLoad)
        rospy.Subscriber('/marker_tracker/image_space_pose_msg', ImageSpacePoseMsg, self.callbackMarkers)
        pub = rospy.Publisher('/keyboard_input', UInt32, queue_size=10)
        pub_carrot = rospy.Publisher('/carrot_point', Float64MultiArray, queue_size=10)

        rospy.init_node('track', anonymous=True)
        rate = rospy.Rate(self.freq) # 15hz
        last_checkpoint = np.copy(self.obj_pos)
        while not rospy.is_shutdown():
            pub_carrot.publish(data=carrot_point)

            if self.ready:
                carrot_point = self.path[iter,:]
            if self.enable and self.ready:
                
                print(iter)

                print('obj. pos: ' + str(self.obj_pos) + ', waypoint: ' + str(carrot_point) + ', dist.: (' + str(np.linalg.norm(self.obj_pos-carrot_point)) + ', ' + str(np.linalg.norm(self.obj_pos-last_checkpoint)) + ', ' + str(np.linalg.norm(self.obj_pos-self.path[iter+1,:]))) + ')'
                print('Last checkpoint: ' + str(last_checkpoint))
                if np.linalg.norm(self.obj_pos-carrot_point) < 1.0 or np.linalg.norm(self.obj_pos-last_checkpoint) > np.linalg.norm(self.obj_pos-self.path[iter+1,:])*1.6:
                    iter += 1
                    last_checkpoint = np.copy(carrot_point)
                    print('******** Moved to next waypoint: ' + str(self.path[iter,:]))

                carrot_point = self.path[iter,:]
                a = self.choose_action_client(carrot_point)
                print('Publishing action: ' + str(a))
                pub.publish(a)
                rospy.sleep(0*1/self.freq+0.1)
                pub.publish(self.KEY_S)
                rospy.sleep(0.25)
                if iter+1==self.path.shape[0] and np.linalg.norm(self.obj_pos-self.path[-1]) < 1: #iter+1==self.path.shape[0] or 
                    print('Reached goal!!!')
                    self.enable = False

            # plt.show()
            # rospy.spin()
            rate.sleep()

if __name__ == '__main__':
    
    try:
        track()
    except rospy.ROSInterruptException:
        pass