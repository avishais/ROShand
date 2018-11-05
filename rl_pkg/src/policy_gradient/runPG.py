#!/usr/bin/env python

import rospy
import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import String
from rl_pkg.srv import net_eval, observation, IsDropped, TargetAngles
from std_srvs.srv import Empty, EmptyResponse
from policy_gradient import PolicyGradient 


class runPG():
    n_inputs = 4
    n_outputs = 4 # right and left for each finger
    # n_outputs = 8 # right, left and stop for each finger

    net = 0
    X = 0
    A = np.array([[-1, -1], [-1, 1], [1, -1], [1, 1], [0, -1], [0, 1], [-1, 0], [1, 0]])

    mode = 5
    reward_mode = 2

    R = []

    gripper_closed = False
    stLearning = True
    possible_plot = False

    def __init__(self):
        rospy.init_node('runPG', anonymous=True)

        if self.mode == 5:
            self.n_inputs = 4
        if self.mode == 8:
            self.n_inputs = 8
        
        self.RL = PolicyGradient(
            n_actions = self.n_outputs,
            n_features = self.n_inputs,
            learning_rate=0.02,
            reward_decay=0.99,
            load_saved_net=False,
            # output_graph=True,
        )

        rospy.Subscriber('/RL/gripper_status', String, self.callbackGripperStatus)
        rospy.Service('/RL/net', net_eval, self.EvalNet)
        rospy.Service('/RL/start_learning', Empty, self.start_learning)
        obs_srv = rospy.ServiceProxy('/RL/observation', observation)
        drop_srv = rospy.ServiceProxy('/RL/IsObjDropped', IsDropped)
        move_srv = rospy.ServiceProxy('/RL/MoveGripper', TargetAngles)
        open_srv = rospy.ServiceProxy('/RL/OpenGripper', Empty)
        close_srv = rospy.ServiceProxy('/RL/CloseGripper', Empty)

        rospy.sleep(3)
        o = open_srv()

        episode_count = 0
        rate = rospy.Rate(15) # 15hz
        while not rospy.is_shutdown():

            if self.stLearning:
                ## Start episode ##
                episode_count += 1

                # Close gripper
                raw_input("Place object between fingers and press Enter to close gripper...")
                close_srv()
                while not self.gripper_closed:
                    rate.sleep()

                raw_input("Remove table and press Enter to start episode...")

                # Get observation
                obs = np.array(obs_srv().state)
                self.VT = []
                while True:
                    # Choose action
                    action = self.RL.choose_action(obs)

                    # Act
                    suc = move_srv(self.A[action])
                    rospy.sleep(0.05)
                    rate.sleep()

                    if suc:
                        # Get observation
                        obs_ = np.array(obs_srv().state)
                        fail = drop_srv().dropped # Check if dropped - end of episode
                    else:
                        # End episode if overload or angle limits reached
                        rospy.logerr('[RL] Failed to move gripper. Episode declared failed.')
                        fail = True 

                    reward, done = self.transition_reward(obs_, fail)

                    self.RL.store_transition(obs, action, reward)

                    obs = obs_

                    if done:
                        ep_rs_sum = sum(self.RL.ep_rs)

                        if 'running_reward' not in globals():
                            running_reward = ep_rs_sum
                        else:
                            running_reward = running_reward * 0.99 + ep_rs_sum * 0.01
                        print("*** episode: " + str(episode_count) + ", episode reward: " + str(ep_rs_sum) + ", running reward: " + str(int(running_reward)) + " ***")

                        vt = self.RL.learn()
                        self.R.append(running_reward)
                        self.possible_plot = True

                        break

                    rate.sleep()
            elif self.possible_plot:
                self.plot_sav()
                self.possible_plot = False

            # Open gripper
            if self.gripper_closed:
                o = open_srv()
                rospy.sleep(0.2)
            # self.stLearning = False

            # print(obs_srv().state)

            # rospy.spin()
            rate.sleep()

    def plot_sav(self):
        plt.plot(range(len(self.R)),self.R)    # plot the episode vt
        plt.xlabel('episode steps')
        plt.ylabel('normalized state-action value')
        plt.show()

    def EvalNet(self, msg):
        a = 0
        return {'action': a}

    def callbackGripperStatus(self, msg):
        self.gripper_closed = msg.data == "closed"

    def start_learning(self, msg):
        self.stLearning = not self.stLearning

        return EmptyResponse()

    def transition_reward(self, obs, fail):

        # Keep moving as much as possible
        if self.reward_mode == 1:
            if fail:
                reward = 0.
            else:
                reward = 1.
            done = fail

        # Get to a certain coodrinate
        if self.reward_mode == 2:
            if fail:
                reward = -3.
            else:
                reward = -1.
            done = fail
            
            if obs[0] > 135.:
                raw_input('Reached goal, x = %f.' % obs[0])
                reward = 5.
                done = True

        return reward, done











if __name__ == '__main__':
    
    try:
        runPG()
    except rospy.ROSInterruptException:
        pass
