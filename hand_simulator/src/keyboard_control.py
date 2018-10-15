#!/usr/bin/python 

'''
----------------------------
Author: Avishai Sintov
        Rutgers University
Date: October 2018
----------------------------
'''


import rospy
import numpy as np 
from std_msgs.msg import Char
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from hand_simulator.srv import MoveServos
from std_srvs.srv import Empty, EmptyResponse
import tty, termios, sys

class keyboard_control():

    fingers_angles = np.array([0.,0.,0.,0.]) # left_proximal, left_distal, right_proximal, right_distal

    num_fingers = 2

    dq = 0.0003

    def __init__(self):
        rospy.init_node('keyboard_control', anonymous=True)

        self.pub_key = rospy.Publisher('/hand/key', Char, queue_size=10)
        rospy.Subscriber('/gripper/pos', Float32MultiArray, self.ActPosCallback)
        move_srv = rospy.ServiceProxy('MoveServos', MoveServos)
        lift_srv = rospy.ServiceProxy('LiftHand', Empty)
        reset_srv = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        self.act_angles = np.zeros(self.num_fingers) # Normalized actuators angles [0,1]

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            ch = self.getchar()
            print ch
            print self.act_angles
            self.pub_key.publish(ch)

            if ord(ch) == 27:
                break

            k = self.Move(ch)
            if all(k == np.array([-100.,-100.])):
                lift_srv()
            elif all(k == np.array([-200.,-200.])):
                move_srv(np.array([0.,0.]))
                reset_srv()
                lift_srv()
            else:
                move_srv(k)
            
            rate.sleep()

    def getchar(self):
   #Returns a single character from standard input
        
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def ActPosCallback(self, msg):
        self.act_angles = np.array(msg.data)

    def Move(self, ch):

        if ch == 's': # Don't move
            return self.act_angles
        if ch == 'x': # Down
            return self.act_angles + np.array([self.dq,self.dq])            
        if ch == 'w': # Up
            return self.act_angles - np.array([self.dq,self.dq])    
        if ch == 'a': # Left
            return self.act_angles + np.array([-self.dq,self.dq])
        if ch == 'd': # Right
            return self.act_angles + np.array([self.dq,-self.dq])
        if ch == 'c': # Down-Right
            return self.act_angles + np.array([self.dq,0])
        if ch == 'z': # Down-left
            return self.act_angles + np.array([0,self.dq])
        if ch == 'e': # Up-Right
            return self.act_angles + np.array([0,-self.dq])
        if ch == 'q': # Up-Left
            return self.act_angles + np.array([-self.dq,0])
        
        if ch == '[': # Close
            return np.array([0.04,0.04])
        if ch == ']': # Open
            return np.array([0.0,0.0])
        if ch == 'r': # Reset
            return np.array([-200.,-200.])
        
        if ch == 'p': # Lift
            return np.array([-100.,-100.])


if __name__ == '__main__':

    try:
        keyboard_control()
    except rospy.ROSInterruptException:
        pass

    

