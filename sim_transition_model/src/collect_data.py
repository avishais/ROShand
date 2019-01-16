#!/usr/bin/env python

import rospy
import numpy as np
import time
import random
from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import Empty, EmptyResponse
from rl_pkg.srv import net_eval, observation, IsDropped, TargetAngles
from transition_experience import *


class collect_data():

    gripper_closed = False
    stCollecting = True # Enable collection
    discrete_actions = True # Discrete or continuous actions

    num_episodes = 20000
    episode_length = 10000

    texp = transition_experience(Load=True, discrete = discrete_actions)

    def __init__(self):
        rospy.init_node('collect_data', anonymous=True)

        rospy.Subscriber('/RL/gripper_status', String, self.callbackGripperStatus)
        pub_gripper_action = rospy.Publisher('RL/gripper_action', Float32MultiArray, queue_size=10)
        rospy.Service('/RL/start_collecting', Empty, self.start_collecting)
        obs_srv = rospy.ServiceProxy('/RL/observation', observation)
        drop_srv = rospy.ServiceProxy('/RL/IsObjDropped', IsDropped)
        move_srv = rospy.ServiceProxy('/RL/MoveGripper', TargetAngles)
        reset_srv = rospy.ServiceProxy('/RL/ResetGripper', Empty)

        rospy.sleep(1)

        msg = Float32MultiArray()

        rate = rospy.Rate(15) # 15hz
        while not rospy.is_shutdown():

            if self.stCollecting:
                
                for n in range(self.num_episodes):

                    # Reset gripper
                    reset_srv()
                    while not self.gripper_closed:
                        rate.sleep()

                    print('[collect_data] Episode %d (%d points so far).'%(n+1, self.texp.getSize()))

                    Done = False

                    # Start episode
                    for ep_step in range(self.episode_length):

                        # Get observation and choose action
                        state = np.array(obs_srv().state)
                        action = self.choose_action()

                        if np.random.uniform() > 0.5:
                            if np.random.uniform() > 0.6:
                                num_steps = np.random.randint(200)
                            else:
                                num_steps = np.random.randint(80)
                        else:
                            num_steps = np.random.randint(20)
                        
                        for _ in range( num_steps ):
                            msg.data = action
                            pub_gripper_action.publish(msg)
                            suc = move_srv(action).success
                            rospy.sleep(0.2)
                            rate.sleep()

                            # Get observation
                            next_state = np.array(obs_srv().state)

                            if suc:
                                fail = drop_srv().dropped # Check if dropped - end of episode
                            else:
                                # End episode if overload or angle limits reached
                                rospy.logerr('[RL] Failed to move gripper. Episode declared failed.')
                                fail = True

                            self.texp.add(state, action, next_state, not suc or fail)
                            state = next_state

                            if not suc or fail:
                                Done = True
                                break

                        if Done:
                            break

                    if n == self.num_episodes-1 or self.texp.getSize() >= 5.0e6:
                        self.stCollecting = False
                        print('Finished running %d episodes!' % self.num_episodes)

                    if n > 0 and n % 20 == 0:
                        self.texp.save()
                
                self.texp.save()             

            rate.sleep()

    def callbackGripperStatus(self, msg):
        self.gripper_closed = msg.data == "closed"

    def start_collecting(self, msg):
        self.stCollecting = not self.stCollecting

    def choose_action(self):
        if self.discrete_actions:
            A = np.array([[1.,1.],[-1.,-1.],[-1.,1.],[1.,-1.],[1.,0.],[-1.,0.],[0.,-1.],[0.,1.]])
            a = A[np.random.randint(A.shape[0])]
            if np.random.uniform(0,1,1) > 0.5:
                if np.random.uniform(0,1,1) > 0.5:
                    a = A[0]
                else:
                    a = A[1]
        else:
            a = np.random.uniform(-1.,1.,2)
            if np.random.uniform(0,1,1) > 0.35:
                if np.random.uniform(0,1,1) > 0.5:
                    a[0] = np.random.uniform(-1.,-0.8,1)
                    a[1] = np.random.uniform(-1.,-0.8,1)
                else:
                    a[0] = np.random.uniform(0.8,1.,1)
                    a[1] = np.random.uniform(0.8,1.,1)

        return a



if __name__ == '__main__':
    
    try:
        collect_data()
    except rospy.ROSInterruptException:
        pass
