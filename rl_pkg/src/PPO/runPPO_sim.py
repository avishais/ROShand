#!/usr/bin/env python

# #!/usr/bin/python3

import rospy
import numpy as np
import tensorflow as tf
from policy_net import Policy_net
from ppo import PPOTrain
from std_msgs.msg import String, Float32MultiArray
from rl_pkg.srv import net_eval, observation, IsDropped, TargetAngles
from std_srvs.srv import Empty, EmptyResponse
import time

GAMMA = 0.95

np.random.seed(1)
tf.set_random_seed(time.time())

class runPPO():
    n_inputs = 4
    # n_outputs = 4 # right and left for each finger
    n_outputs = 8 # right, left and stop for each finger
    A = np.array([[-1, -1], [1, -1], [-1, 1], [1, 1], [0, -1], [0, 1], [-1, 0], [1, 0]]) # Set of action

    max_episodes = 1000
    max_steps = 1000
    stop_bound = 800

    g = np.array([-35.0, 104.0], dtype='f') # Goal
    prev_dis2goal = 1e9

    gripper_closed = False
    stLearning = True # Enable learning
    reward_mode = 3

    def __init__(self):
        rospy.init_node('runPPO', anonymous=True)

        Policy = Policy_net('policy', self.n_inputs, self.n_outputs)
        Old_Policy = Policy_net('old_policy', self.n_inputs, self.n_outputs)
        PPO = PPOTrain(Policy, Old_Policy, gamma=GAMMA, c_2 = 0.1)
        saver = tf.train.Saver()

        rospy.Subscriber('/RL/gripper_status', String, self.callbackGripperStatus)
        # rospy.Service('/RL/net', net_eval, self.EvalNet)
        rospy.Service('/RL/start_learning', Empty, self.start_learning)
        obs_srv = rospy.ServiceProxy('/RL/observation', observation)
        drop_srv = rospy.ServiceProxy('/RL/IsObjDropped', IsDropped)
        move_srv = rospy.ServiceProxy('/RL/MoveGripper', TargetAngles)
        reset_srv = rospy.ServiceProxy('/RL/ResetGripper', Empty)
        pub_goal = rospy.Publisher('/RL/Goal', Float32MultiArray, queue_size=10)

        gg = Float32MultiArray()
        gg.data = self.g

        with tf.Session() as sess:
            # $ tensorboard --logdir=logs
            # http://0.0.0.0:6006/
            writer = tf.summary.FileWriter('/home/pracsys/catkin_ws/src/rutgers_collab/src/rl_pkg/src/PPO/log/train', sess.graph)

            sess.run(tf.global_variables_initializer())
            reward = 0
            success_num = 0

            episode_count = 0
            rate = rospy.Rate(100) # 100hz
            while not rospy.is_shutdown():

                if self.stLearning:
                    ## Start episode ##
                    episode_count += 1                

                    # Reset gripper
                    reset_srv()
                    while not self.gripper_closed:
                        rate.sleep()

                    # Get observation
                    obs = np.array(obs_srv().state)
                    self.prev_dis2goal = np.linalg.norm(self.g-obs[:2])

                    observations = []
                    actions = []
                    v_preds = []
                    rewards = []
                    run_policy_steps = 0
                    while True:  # run policy RUN_POLICY_STEPS which is much less than episode length
                        run_policy_steps += 1
                        print('[RL] Step %d in episode %d, distance to goal: %f.' % (run_policy_steps, episode_count, self.prev_dis2goal))

                        pub_goal.publish(gg)

                        obs = np.stack([obs]).astype(dtype=np.float32)  # prepare to feed placeholder Policy.obs

                        while 1:
                            act, v_pred = Policy.act(obs=obs, stochastic=True)
                            act = np.asscalar(act)
                            v_pred = np.asscalar(v_pred)
                            if act < 8:
                                break     

                        # Act
                        suc = move_srv(self.A[act])
                        rospy.sleep(0.05)
                        rate.sleep()

                        if suc:
                            # Get observation
                            next_obs = np.array(obs_srv().state)
                            fail = drop_srv().dropped # Check if dropped - end of episode
                        else:
                            # End episode if overload or angle limits reached
                            rospy.logerr('[RL] Failed to move gripper. Episode declared failed.')
                            fail = True 

                        reward, done = self.transition_reward(next_obs, fail)

                        observations.append(obs)
                        actions.append(act)
                        v_preds.append(v_pred)
                        rewards.append(reward) # Weird that this is before the step - this is the reward of the previos action

                        print('[RL] Action %d yielded reward %f and position (%f,%f).' % (act, reward, obs[0][0], obs[0][1]))

                        if run_policy_steps > self.max_steps:
                            done = True

                        if done:
                            v_preds_next = v_preds[1:] + [0]  # next state of terminate state has 0 state value - adds zero in the end of the vector
                            
                            reward = -1
                            break
                        else:
                            obs = next_obs

                        rate.sleep()

                    print('episode_length', run_policy_steps, 'episode_reward', sum(rewards))

                    writer.add_summary(tf.Summary(value=[tf.Summary.Value(tag='episode_length', simple_value=run_policy_steps)])
                                    , episode_count)
                    writer.add_summary(tf.Summary(value=[tf.Summary.Value(tag='episode_reward', simple_value=sum(rewards))])
                                    , episode_count)

                    if sum(rewards) >= self.stop_bound:
                        success_num += 1
                        if success_num >= 100:
                            saver.save(sess, '/home/pracsys/catkin_ws/src/rutgers_collab/src/rl_pkg/logs/model_ppo.ckpt')
                            print('Clear!! Model saved.')
                            break
                    else:
                        success_num = 0

                    gaes = PPO.get_gaes(rewards=rewards, v_preds=v_preds, v_preds_next=v_preds_next)

                    # convert list to numpy array for feeding tf.placeholder
                    observations = np.reshape(observations, newshape=[-1] + list((self.n_inputs,)))
                    actions = np.array(actions).astype(dtype=np.int32)
                    rewards = np.array(rewards).astype(dtype=np.float32)
                    v_preds_next = np.array(v_preds_next).astype(dtype=np.float32)
                    gaes = np.array(gaes).astype(dtype=np.float32)
                    gaes = (gaes - gaes.mean()) / gaes.std()

                    PPO.assign_policy_parameters()

                    inp = [observations, actions, rewards, v_preds_next, gaes]

                    # train
                    for epoch in range(4):
                        sample_indices = np.random.randint(low=0, high=observations.shape[0], size=64)  # indices are in [low, high)
                        sampled_inp = [np.take(a=a, indices=sample_indices, axis=0) for a in inp]  # sample training data
                        PPO.train(obs=sampled_inp[0],
                                actions=sampled_inp[1],
                                rewards=sampled_inp[2],
                                v_preds_next=sampled_inp[3],
                                gaes=sampled_inp[4])

                    summary = PPO.get_summary(obs=inp[0],
                                            actions=inp[1],
                                            rewards=inp[2],
                                            v_preds_next=inp[3],
                                            gaes=inp[4])[0]

                    writer.add_summary(summary, episode_count)

                if episode_count > self.max_episodes:
                    break

                rate.sleep()
                
            writer.close()

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
            reward = self.prev_dis2goal - d

            # if fail or d > self.prev_dis2goal:
            #     reward = 0
            # else:
            #     reward = 38./d
            done = fail
            
            if d < 5:
                print('Reached goal, (x,y) = (%f,%f).' % (obs[0],obs[1]))
                reward = 50.
                done = True
            
            self.prev_dis2goal = d

        return reward, done


if __name__ == '__main__':
    
    try:
        runPPO()
    except rospy.ROSInterruptException:
        pass
