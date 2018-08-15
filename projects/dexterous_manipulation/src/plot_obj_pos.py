#!/usr/bin/env python
import rospy
from marker_tracker.msg import ImageSpacePoseMsg
import numpy as np
from matplotlib import pyplot as plt

class Plot():

    counter = 0
    obj_pos = np.array([0,0])
    start_pos = np.array([0,0])
    freq = 5

    def callbackMarkers(self, msg):
        try:
            self.obj_pos = np.array([msg.posx[msg.ids.index(5)], msg.posy[msg.ids.index(5)]])
            if self.counter % 10 == 0:
                plt.plot(self.obj_pos[0], self.obj_pos[1],'.r')
                plt.plot(self.start_pos[0], self.start_pos[1],'*b')
                plt.axis("equal")
                plt.axis([200, 750, 90, 350])
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


    def __init__(self):
        
        rospy.Subscriber('/marker_tracker/image_space_pose_msg', ImageSpacePoseMsg, self.callbackMarkers)

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