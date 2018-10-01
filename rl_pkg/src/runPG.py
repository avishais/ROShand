#!/usr/bin/env python

import rospy
import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
from rl_pkg.srv import net_eval, observation
from policy_gradient import PolicyGradient 


class runPG():
    n_inputs = 4
    n_outputs = 4 # right and left for each finger

    net = 0
    X = 0

    mode = 5

    def __init__(self):
        rospy.init_node('runPG', anonymous=True)

        if self.mode == 5:
            self.n_inputs = 4
        if self.mode == 8:
            self.n_inputs = 8
        
        self.RL = PolicyGradient(
            n_actions = self.n_inputs,
            n_features = self.n_outputs,
            learning_rate=0.02,
            reward_decay=0.99,
            # output_graph=True,
        )

        rospy.Service('/RL/net', net_eval, self.EvalNet)
        obs_srv = rospy.ServiceProxy('/RL/observation', observation)
        obs_srv = rospy.ServiceProxy('/RL/IsObjDropped', IsDropped)

        # Restore variables from disk.
        # model_file = "/home/pracsys/Documents/workspace/adaptive_hand_model/nn/models/cc_8.ckpt"
        # saver = tf.train.Saver()
        # saver.restore(self.sess, model_file)

        rate = rospy.Rate(15) # 15hz
        while not rospy.is_shutdown():

            obs = np.array(obs_srv().state)
            # obs = np.reshape(obs, (1, self.n_inputs))
            # print(obs, obs.shape)

            a = self.RL.choose_action(obs)
            print(obs, a)

            # rospy.spin()
            rate.sleep()

    def EvalNet(self, msg):
        a = 0
        

        return {'action': a}








if __name__ == '__main__':
    
    try:
        runPG()
    except rospy.ROSInterruptException:
        pass
