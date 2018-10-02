#!/usr/bin/python 

import rospy
import numpy as np 
from std_msgs.msg import Float64MultiArray, Float32MultiArray, String
from std_srvs.srv import Empty, EmptyResponse
from rl_pkg.srv import TargetAngles, IsDropped, observation
from openhand.srv import MoveServos
from marker_tracker.msg import ImageSpacePoseMsg
import math

class hand_control():

    finger_initial_offset = np.array([0., 0.])
    finger_opening_position = np.array([0.2, 0.33])
    finger_closing_position = np.array([0., 0.])
    finger_move_offset = np.array([0.01, 0.01])
    closed_load = np.array(20.)

    gripper_pos = np.array([0., 0.])
    gripper_load = np.array([0., 0.])
    base_pos = [0,0]
    base_theta = 0
    obj_pos = [0,0]
    R = []
    count = 1

    gripper_status = 'open'

    move_servos_srv = 0.

    def __init__(self):
        rospy.init_node('hand_control', anonymous=True)
        
        if rospy.has_param('~finger_initial_offset'):
            self.finger_initial_offset = rospy.get_param('~finger_initial_offset')
            self.finger_opening_position = rospy.get_param('~finger_opening_position')
            self.finger_closing_position = rospy.get_param('~finger_closing_position')
            self.finger_move_offset = rospy.get_param('~finger_move_offset')
            self.closed_load = rospy.get_param('~finger_close_load')

        rospy.Subscriber('/gripper/pos', Float32MultiArray, self.callbackGripperPos)
        rospy.Subscriber('/gripper/load', Float32MultiArray, self.callbackGripperLoad)
        rospy.Subscriber('/marker_tracker/image_space_pose_msg', ImageSpacePoseMsg, self.callbackMarkers)
        pub_gripper_status = rospy.Publisher('/RL/gripper_status', String, queue_size=10)

        rospy.Service('/RL/OpenGripper', Empty, self.OpenGripper)
        rospy.Service('/RL/CloseGripper', Empty, self.CloseGripper)
        rospy.Service('/RL/MoveGripper', TargetAngles, self.MoveGripper)
        rospy.Service('/RL/IsObjDropped', IsDropped, self.CheckDropped)
        rospy.Service('/RL/observation', observation, self.GetObservation)

        self.move_servos_srv = rospy.ServiceProxy('/MoveServos', MoveServos)

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
            self.obj_pos = np.array([np.nan, np.nan])

    def OpenGripper(self, msg):
        self.moveGripper(self.finger_opening_position)

        self.gripper_status = 'open'

        return EmptyResponse()

    def CloseGripper(self, msg):

        for i in range(100):
            # print('Angles: ' + str(self.gripper_pos) + ', load: ' + str(self.gripper_load), self.closed_load)
            if abs(self.gripper_load[0]) > self.closed_load or abs(self.gripper_load[1]) > self.closed_load:
                rospy.loginfo('[RL] Object grasped.')
                self.gripper_status = 'closed'
                break

            desired = self.gripper_pos + self.finger_move_offset
            if desired[0] > 0.55 or desired[1] > 0.55:
                rospy.logerr('[RL] Desired angles out of bounds.')
                break
            self.moveGripper(desired)
            rospy.sleep(0.2)  

        return EmptyResponse()    


    def MoveGripper(self, msg):
        # This function should accept a vector of normalized incraments to the current angles: msg.angles = [dq1, dq2], where dq1 and dq2 can be equal to 0 (no move), 1,-1 (increase or decrease angles by finger_move_offset)

        inc = np.array(msg.angles)
        inc_angles = np.multiply(self.finger_move_offset, inc)

        desired = self.gripper_pos + inc_angles

        suc = self.moveGripper(desired)

        return {'success': suc}
    
    def moveGripper(self, angles):
        if angles[0] > 0.7 or angles[1] > 0.7 or angles[0] < 0.1 or angles[1] < 0.1:
            rospy.logerr('[RL] Desired angles out of bounds.')
            return False

        if abs(self.gripper_load[0]) > 300 or abs(self.gripper_load[1]) > 300:
            rospy.logerr('[RL] Pre-overload.')
            return False

        self.move_servos_srv.call(angles)

        return True

    def CheckDropped(self, msg):

        if np.isnan(self.obj_pos[0]) and np.isnan(self.obj_pos[1]):
            rospy.loginfo('[RL] Object not visible - assumed dropped.')
            return {'dropped': True}

        return {'dropped': False}

    def GetObservation(self, msg):
        obs = np.concatenate((self.obj_pos, self.gripper_load))

        return {'state': obs}



if __name__ == '__main__':
    
    try:
        hand_control()
    except rospy.ROSInterruptException:
        pass