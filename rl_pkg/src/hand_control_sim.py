#!/usr/bin/python 

import rospy
import numpy as np 
from std_msgs.msg import Float64MultiArray, Float32MultiArray, String
from std_srvs.srv import Empty, EmptyResponse
from rl_pkg.srv import TargetAngles, IsDropped, observation
from openhand.srv import MoveServos
import math

class hand_control():

    finger_opening_position = np.array([0.005, 0.005])
    finger_closing_position = np.array([0.08, 0.08])
    finger_move_offset = np.array([0.0002, 0.0002])
    closed_load = np.array(20.)

    gripper_pos = np.array([0., 0.])
    gripper_load = np.array([0., 0.])
    lift_status = False # True - lift up, False - lift down
    object_grasped = False
    base_pos = [0,0]
    base_theta = 0
    obj_pos = [0,0]
    R = []
    count = 1

    gripper_status = 'open'

    move_servos_srv = 0.

    def __init__(self):
        rospy.init_node('hand_control_sim', anonymous=True)
        
        rospy.Subscriber('/gripper/pos', Float32MultiArray, self.callbackGripperPos)
        rospy.Subscriber('/gripper/load', Float32MultiArray, self.callbackGripperLoad)
        rospy.Subscriber('/gripper/lift_status', String, self.callbackLiftStatus)
        rospy.Subscriber('/hand/obj_pos', Float32MultiArray, self.callbackObj)
        pub_gripper_status = rospy.Publisher('/RL/gripper_status', String, queue_size=10)

        rospy.Service('/RL/ResetGripper', Empty, self.ResetGripper)
        rospy.Service('/RL/MoveGripper', TargetAngles, self.MoveGripper)
        rospy.Service('/RL/IsObjDropped', IsDropped, self.CheckDropped)
        rospy.Service('/RL/observation', observation, self.GetObservation)

        self.move_servos_srv = rospy.ServiceProxy('/MoveServos', MoveServos)
        self.move_lift_srv = rospy.ServiceProxy('/LiftHand', Empty)
        self.reset_srv = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        #### Later I should remove the angles from hands.py and set initial angles here at the start ####

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            pub_gripper_status.publish(self.gripper_status)

            # print(self.obj_pos)
            # rospy.spin()
            rate.sleep()

    def callbackGripperPos(self, msg):
        self.gripper_pos = np.array(msg.data)

    def callbackGripperLoad(self, msg):
        self.gripper_load = np.array(msg.data)

    def callbackLiftStatus(self, msg):
        self.lift_status = True if msg.data == 'up' else False

    def callbackObj(self, msg):
        self.obj_pos = np.array(msg.data)

        self.object_grasped = True if abs(self.obj_pos[2]) < 1e-2 else False

        self.obj_pos = self.obj_pos[:2]*1000 # m to mm

    def ResetGripper(self, msg):
        ratein = rospy.Rate(15)
        while 1:
            # Open gripper
            self.moveGripper(self.finger_opening_position)
            # Drop lift down
            while self.lift_status:
                self.move_lift_srv.call()
                rospy.sleep(.1)
                ratein.sleep()
            # Reset object
            rospy.sleep(.2)
            self.reset_srv.call()
            # Close gripper
            self.moveGripper(self.finger_closing_position)
            rospy.sleep(0.7)
            self.move_lift_srv.call()
            rospy.sleep(0.2)
            ratein.sleep()
            if self.object_grasped:# and self.lift_status:
                break
        
        self.gripper_status = 'closed'

        return EmptyResponse()

    def MoveGripper(self, msg):
        # This function should accept a vector of normalized incraments to the current angles: msg.angles = [dq1, dq2], where dq1 and dq2 can be equal to 0 (no move), 1,-1 (increase or decrease angles by finger_move_offset)

        inc = np.array(msg.angles)
        inc_angles = np.multiply(self.finger_move_offset, inc)

        desired = self.gripper_pos + inc_angles

        suc = self.moveGripper(desired)

        return {'success': suc}
    
    def moveGripper(self, angles):
        if angles[0] > 0.7 or angles[1] > 0.7 or angles[0] < 0.003 or angles[1] < 0.003:
            rospy.logerr('[RL] Desired angles out of bounds.')
            return False

        if abs(self.gripper_load[0]) > 120 or abs(self.gripper_load[1]) > 120:
            rospy.logerr('[RL] Pre-overload.')
            return False

        self.move_servos_srv.call(angles)

        return True

    def CheckDropped(self, msg):

        return {'dropped': not self.object_grasped}

    def GetObservation(self, msg):
        obs = np.concatenate((self.obj_pos, self.gripper_load))

        return {'state': obs}



if __name__ == '__main__':
    
    try:
        hand_control()
    except rospy.ROSInterruptException:
        pass