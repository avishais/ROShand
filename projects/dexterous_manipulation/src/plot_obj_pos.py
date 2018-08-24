#!/usr/bin/env python
import rospy
from marker_tracker.msg import ImageSpacePoseMsg
from sensor_msgs.msg import Image
from action_node.srv import empty
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.image as mpimg
import cv2
from cv_bridge import CvBridge, CvBridgeError

class Plot():

    counter = 0
    obj_pos = np.array([0,0])
    base_pos = np.array([0,0])
    start_pos = np.array([0,0])
    im = np.array([])
    base_theta = 0
    freq = 15
    ref_data = np.array([])
    image_exists = False
    with_image = False

    def callbackMarkers(self, msg):
        try:
            self.obj_pos = np.array([msg.posx[msg.ids.index(5)], msg.posy[msg.ids.index(5)]])
            self.base_pos = np.array([msg.posx[msg.ids.index(0)], msg.posy[msg.ids.index(0)]])
            self.base_theta = 0#np.pi - msg.angles[msg.ids.index(0)]

            if self.with_image and self.image_exists and self.counter % 5 == 0:
                plt.imshow(self.im)
            if self.counter % 10 == 0:
                if self.with_image:
                    plt.plot(self.obj_pos[0], self.obj_pos[1],'.r')
                    plt.plot(self.start_pos[0], self.start_pos[1],'*b')
                else:
                    plt.plot(self.obj_pos[0], -self.obj_pos[1],'.r')
                    plt.plot(self.start_pos[0], -self.start_pos[1],'*b')
                    plt.axis("equal")
                    plt.axis([200, 750, -350, -90])

                # print(self.ref_data)
                # plt.plot(self.ref_data[:,0], self.ref_data[:,1],'xg')
                plt.xlabel('x')
                plt.ylabel('y')
                plt.title('Real-time object position')
                plt.draw()
                plt.pause(0.00000000001)
            self.counter += 1

            if self.counter <= 5:
                self.start_pos = self.obj_pos
        except:
            pass

    def callbackPlotRefPath(self, req):

        fname = "/home/pracsys/catkin_ws/src/rutgers_collab/src/planner/paths/path.txt"

        self.ref_data = np.loadtxt(fname,delimiter=' ', usecols=range(4))
        self.ref_data = self.ref_data[:,:2]

        R = np.array([[np.cos(self.base_theta), -np.sin(self.base_theta)], [np.sin(self.base_theta), np.cos(self.base_theta)]])

        # print('base', self.base_pos, self.base_theta)
        # print(self.ref_data)

        # Reproject to image plane
        for i in range(self.ref_data.shape[0]):
            self.ref_data[i,:] = np.matmul(R.T, self.ref_data[i,:].T) + self.base_pos

        # print(self.ref_data)

        return {}

    def image_callback(self, msg):

        # try:
        if self.with_image:# and self.counter % 100 == 0:
            self.im = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.im = cv2.cvtColor(self.im, cv2.COLOR_BGR2RGB)
            self.image_exists = True
        # except CvBridgeError as e:
        #     print(e)


        # plt.imshow(self.im)
        # plt.show()
        



    def __init__(self):
        
        rospy.Subscriber('/marker_tracker/image_space_pose_msg', ImageSpacePoseMsg, self.callbackMarkers)
        sr = rospy.Service('/plot_ref_path', empty, self.callbackPlotRefPath)
        image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()

        rospy.init_node('Plot_obj_pos', anonymous=True)
        rate = rospy.Rate(self.freq) # 15hz
        while not rospy.is_shutdown():
            # plt.show()
            rospy.spin()
            # rate.sleep()


if __name__ == '__main__':
    
    try:
        Plot()
    except rospy.ROSInterruptException:
        pass