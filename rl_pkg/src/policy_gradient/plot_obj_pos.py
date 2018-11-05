#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from action_node.srv import empty
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.image as mpimg
from std_msgs.msg import Float32MultiArray



class Plot():

    counter = 0
    obj_pos = np.array([0.,0.])
    start_pos = np.array([0.,0.])
    freq = 15
    goal = np.array([0.,0.])

    def callbackObj(self, msg):
        self.obj_pos = np.array(msg.data)

        self.obj_pos = self.obj_pos[:2]*1000 # m to mm

        if self.counter % 10 == 0:
            plt.gcf().clear()
            plt.plot(self.obj_pos[0], self.obj_pos[1],'.r')
            # plt.plot(self.start_pos[0], self.start_pos[1],'*b')
            plt.plot(self.goal[0], self.goal[1],'*g')
            plt.axis("equal")
            plt.axis([-60, 60, 60, 177])
            plt.xlabel('x')
            plt.ylabel('y')
            plt.title('Real-time object position:' + str(self.obj_pos))
            plt.draw()
            plt.pause(0.00000000001)

        self.counter += 1

        if self.counter <= 5:
            self.start_pos = self.obj_pos

    def callbackGoal(self, msg):
        self.goal = np.array(msg.data)

    def __init__(self):
        
        rospy.Subscriber('/hand/obj_pos', Float32MultiArray, self.callbackObj)
        rospy.Subscriber('/RL/Goal', Float32MultiArray, self.callbackGoal)

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