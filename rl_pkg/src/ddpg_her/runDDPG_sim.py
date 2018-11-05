#!/usr/bin/env python

import rospy
import numpy as np
import tensorflow as tf
import time
import random
from ddpg import Episode_experience, DDPGAgent
from std_msgs.msg import String, Float32MultiArray
from rl_pkg.srv import net_eval, observation, IsDropped, TargetAngles
from std_srvs.srv import Empty, EmptyResponse
import matplotlib.pyplot as plt


class runDDPG():
    n_inputs = 4
    n_outputs = 3 # Actuator angles change
    # state_region = np.array([[-70,70],[80,130]])
    state_region = np.array([[-10,10],[117.5,120]])
    load_saved_net = False

    num_epochs = 1000
    num_episodes = 20
    episode_length = 400
    # stop_bound = 800
    optimization_steps = 20
    K = 15 # number of random future states

    a_losses = []
    c_losses = []
    ep_mean_r = []
    success_rate = []

    ep_experience = Episode_experience()
    ep_experience_her = Episode_experience()

    global_goal = np.array([-35.0, 104.0], dtype='f') # Goal

    gripper_closed = False
    stLearning = True # Enable learning

    variance = 10 # Variance in which to explore the action space (decreases over iterations)
    use_her = True
    threshold = 1.
    reward_type = 'sparse'


    def __init__(self):
        rospy.init_node('runDDPG', anonymous=True)

        agent = DDPGAgent(state_size=self.n_inputs, action_size=self.n_outputs, goal_size=2, actor_learning_rate=1e-4, critic_learning_rate=3e-4, tau=0.1, load_saved_net=self.load_saved_net)

        rospy.Subscriber('/RL/gripper_status', String, self.callbackGripperStatus)
        rospy.Service('/RL/start_learning', Empty, self.start_learning)
        obs_srv = rospy.ServiceProxy('/RL/observation', observation)
        drop_srv = rospy.ServiceProxy('/RL/IsObjDropped', IsDropped)
        move_srv = rospy.ServiceProxy('/RL/MoveGripper', TargetAngles)
        reset_srv = rospy.ServiceProxy('/RL/ResetGripper', Empty)
        pub_goal = rospy.Publisher('/RL/Goal', Float32MultiArray, queue_size=10)

        gg = Float32MultiArray()

        epochs_count = 0
        total_step = 0
        total_episodes = 0

        visited_states = np.array([[0.,0.]])
        reached_goals = np.array([[0.,0.]])
        failed_goals = np.array([[0.,0.]])

        rate = rospy.Rate(100) # 100hz
        while not rospy.is_shutdown():

            if self.stLearning:
                ## Start episode ##
                epochs_count += 1

                successes = 0
                ep_total_r = 0
                for n in range(self.num_episodes):
                    total_episodes += 1

                    # Reset gripper
                    reset_srv()
                    while not self.gripper_closed:
                        rate.sleep()

                    # Get observation
                    state = np.array(obs_srv().state)

                    # Sample goal
                    goal = self.sample_goal()
                    gg.data = goal
                    pub_goal.publish(gg)

                    for ep_step in range(self.episode_length):
                        print('[RL] Step %d in epoch %d, episode %d, state (%f,%f), goal (%f,%f), distance %f.' % (ep_step, epochs_count, n+1, state[0], state[1], goal[0], goal[1], np.linalg.norm(state[:2]-goal)))
                        total_step += 1

                        action = agent.choose_action([state], [goal], self.variance)

                        # Act
                        for _ in range(int(max(np.round((action[2]+1.)/2*10), 1))):
                            suc = move_srv(action[:2])
                            rospy.sleep(0.05)
                            rate.sleep()
                            if not suc:
                                break

                        if suc:
                            # Get observation
                            next_state = np.array(obs_srv().state)
                            fail = drop_srv().dropped # Check if dropped - end of episode
                        else:
                            # End episode if overload or angle limits reached
                            rospy.logerr('[RL] Failed to move gripper. Episode declared failed.')
                            fail = True 

                        visited_states = np.append(visited_states, [next_state[:2]], axis=0)

                        reward, done = self.transition_reward(next_state, goal, fail)
                        ep_total_r += reward
                        self.ep_experience.add(state, action, reward, next_state, done, goal)
                        state = next_state

                        if total_step % 100 == 0 or ep_step==self.episode_length-1:
                            if self.use_her: # The strategy can be changed here
                                for t in range(len(self.ep_experience.memory)):
                                    for _ in range(self.K):
                                        future = np.random.randint(t, len(self.ep_experience.memory))
                                        goal_ = self.ep_experience.memory[future][3][:2] # next_state of future
                                        state_ = self.ep_experience.memory[t][0]
                                        action_ = self.ep_experience.memory[t][1]
                                        next_state_ = self.ep_experience.memory[t][3]
                                        reward_, done_ = self.transition_reward(next_state_, goal_, False)
                                        self.ep_experience_her.add(state_, action_, reward_, next_state_, done_, goal_)
                                agent.remember(self.ep_experience_her)
                                self.ep_experience_her.clear()
                            agent.remember(self.ep_experience)
                            self.ep_experience.clear()
                            self.variance *= 0.9998
                            a_loss, c_loss = agent.replay(self.optimization_steps)
                            self.a_losses += [a_loss]
                            self.c_losses += [c_loss]
                            agent.update_target_net()
                        if done:
                            if reward == 0:
                                print("[RL] Reached goal.")
                                reached_goals = np.append(reached_goals, [goal], axis=0)
                            else:
                                failed_goals = np.append(failed_goals, [goal], axis=0)
                            break

                        rate.sleep()

                    successes += reward>=0 and done

                self.success_rate.append(successes/float(self.num_episodes))
                self.ep_mean_r.append(ep_total_r/float(self.num_episodes))
                print("*** Epoch " + str(epochs_count+1) + ", success rate " + str(self.success_rate[-1]) + ", ep_mean_r %.2f"%self.ep_mean_r[-1] + ", exploration %.2f"%self.variance)
                if len(self.success_rate) > 0 and len(self.ep_mean_r) > 0 and len(self.a_losses) > 0:
                    agent.log2summary(total_episodes, self.success_rate[-1], self.a_losses[-1], self.ep_mean_r[-1])

            else:
                plt.plot(visited_states[1:,0], visited_states[1:,1],'.b', label='visited states')
                plt.plot(failed_goals[1:,0], failed_goals[1:,1],'.r', label='failed goals')
                plt.plot(reached_goals[1:,0], reached_goals[1:,1],'.g', label='reached goals')
                plt.legend(loc='best')
                plt.xlabel('x')
                plt.ylabel('y')
                plt.show()

            
            if epochs_count > self.num_epochs:
                break
            
            rate.sleep()


    def callbackGripperStatus(self, msg):
        self.gripper_closed = msg.data == "closed"

    def start_learning(self, msg):
        self.stLearning = not self.stLearning

    def sample_goal(self):
        goal = np.array([0.,0.])
        for i in range(2):
            goal[i] = np.random.uniform(self.state_region[i][0], self.state_region[i][1])

        return goal

    def transition_reward(self, state, goal, fail):

        state = state[:2]
        good_done = np.linalg.norm(state-goal) <= self.threshold # Reached goal
        bad_done = fail or state[0] < self.state_region[0][0] or state[0] > self.state_region[0][1] or state[1] < self.state_region[1][0] or state[1] > self.state_region[1][1]
        if self.reward_type == 'sparse':
            reward = 0 if good_done else -1 #(-1 if fail else -10) #      -1 # Punish -1 if not reached goal
        else:
            reward = 5*self.size if good_done else -10 if bad_done else -np.linalg.norm(state-goal)/200
        return reward, good_done or bad_done



if __name__ == '__main__':
    
    try:
        runDDPG()
    except rospy.ROSInterruptException:
        pass
