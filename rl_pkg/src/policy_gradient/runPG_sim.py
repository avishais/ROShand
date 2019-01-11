#!/usr/bin/env python

import rospy
import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import String, Float32MultiArray
from rl_pkg.srv import net_eval, observation, IsDropped, TargetAngles
from std_srvs.srv import Empty, EmptyResponse
from policy_gradient import PolicyGradient 


class runPG():
    n_inputs = 4
    # n_outputs = 4 # right and left for each finger
    n_outputs = 8 # right, left and stop for each finger
    max_episodes = 1200
    max_steps = 2500

    net = 0
    X = 0
    A = np.array([[-1, -1], [1, -1], [-1, 1], [1, 1], [0, -1], [0, 1], [-1, 0], [1, 0]])

    mode = 5
    reward_mode = 3

    R = []
    g = np.array([-35.0, 104.0], dtype='f') # Goal

    gripper_closed = False
    stLearning = True # Enable learning
    possible_plot = False

    # For reward mode 3
    prev_dis2goal = 1e9


    def __init__(self):
        rospy.init_node('runPG', anonymous=True)

        if self.mode == 5:
            self.n_inputs = 4
        if self.mode == 8:
            self.n_inputs = 8
        
        self.RL = PolicyGradient(
            n_actions = self.n_outputs,
            n_features = self.n_inputs,
            learning_rate=0.001,
            reward_decay=0.98,
            load_saved_net=True,
            # output_graph=True,
        )

        rospy.Subscriber('/RL/gripper_status', String, self.callbackGripperStatus)
        rospy.Service('/RL/net', net_eval, self.EvalNet)
        rospy.Service('/RL/start_learning', Empty, self.start_learning)
        obs_srv = rospy.ServiceProxy('/RL/observation', observation)
        drop_srv = rospy.ServiceProxy('/RL/IsObjDropped', IsDropped)
        move_srv = rospy.ServiceProxy('/RL/MoveGripper', TargetAngles)
        reset_srv = rospy.ServiceProxy('/RL/ResetGripper', Empty)
        pub_goal = rospy.Publisher('/RL/Goal', Float32MultiArray, queue_size=10)

        gg = Float32MultiArray()
        gg.data = self.g

        episode_count = 0
        rate = rospy.Rate(100) # 100hz
        while not rospy.is_shutdown():
            
            if self.stLearning:
                ## Start episode ##
                episode_count += 1

                self.prev_dis2goal = 1e9

                # Set gripper
                reset_srv()
                while not self.gripper_closed:
                    rate.sleep()

                # Get observation
                obs = np.array(obs_srv().state)
                self.VT = []
                step = 0
                while True:
                    step += 1
                    print('[RL] Step %d in episode %d, distance to goal: %f.' % (step, episode_count, self.prev_dis2goal))
                    pub_goal.publish(gg)

                    # Choose action
                    action = self.RL.choose_action(obs)

                    # Act
                    suc = move_srv(self.A[action]).success
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

                    if step > self.max_steps:
                        done = True

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

            if self.max_episodes < episode_count:
                self.plot_sav()
                break

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

        # Cross a line
        if self.reward_mode == 2:
            if fail:
                reward = -3.
            else:
                reward = -1.
            done = fail
            
            if obs[0] > 40.:
                print('Reached goal, x = %f.' % obs[0])
                reward = 5.
                done = True

        # Get to a certain coordinate
        if self.reward_mode == 3:
            
            d = np.linalg.norm(self.g-obs[:2])

            if fail or d > self.prev_dis2goal:
                reward = 0.
            else:
                reward = 1.
            done = fail
            
            if d < 5:
                print('Reached goal, (x,y) = (%f,%f).' % (obs[0],obs[1]))
                reward = 50.
                done = True
            
            self.prev_dis2goal = d

        return reward, done











if __name__ == '__main__':
    
    try:
        runPG()
    except rospy.ROSInterruptException:
        pass
