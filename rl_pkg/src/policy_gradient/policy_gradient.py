"""
This part of code is the reinforcement learning brain, which is a brain of the agent.
All decisions are made in here.

Policy Gradient, Reinforcement Learning.

View more on my tutorial page: https://morvanzhou.github.io/tutorials/

Using:
Tensorflow: 1.0
gym: 0.8.0
"""

import numpy as np
import tensorflow as tf
import csv
import os.path

# reproducible
# np.random.seed(1)
# tf.set_random_seed(1)


class PolicyGradient:
    def __init__(
            self,
            n_actions,
            n_features,
            learning_rate=0.001,
            reward_decay=0.95,
            load_saved_net=False,
            output_graph=False,
    ):
        self.n_actions = n_actions
        self.n_features = n_features
        self.lr = learning_rate
        self.gamma = reward_decay

        self.ep_obs, self.ep_as, self.ep_rs = [], [], []

        self._build_net()

        self.sess = tf.Session()

        self.iteration = 1

        # Add ops to save and restore all the variables.
        self.saved_net = '/home/pracsys/catkin_ws/src/rutgers_collab/src/rl_pkg/logs/net.ckpt'
        self.saver = tf.train.Saver()

        if output_graph:
            # $ tensorboard --logdir=logs
            # http://0.0.0.0:6006/
            # tf.train.SummaryWriter soon be deprecated, use following
            tf.summary.FileWriter("logs/", self.sess.graph)

        self.sess.run(tf.global_variables_initializer())

        if load_saved_net and os.path.isfile(self.saved_net + '.meta'):
            try:
                self.saver.restore(self.sess, self.saved_net) 
                print('Saved net loaded.')
            except:
                pass

    def _build_net(self):
        with tf.name_scope('inputs'):
            self.tf_obs = tf.placeholder(tf.float32, [None, self.n_features], name="observations")
            self.tf_acts = tf.placeholder(tf.int32, [None, ], name="actions_num")
            self.tf_vt = tf.placeholder(tf.float32, [None, ], name="actions_value") # Rewards
        # fc1
        layer1 = tf.layers.dense(
            inputs=self.tf_obs,
            units=20,
            activation=tf.nn.tanh,  # tanh activation
            kernel_initializer=tf.random_normal_initializer(mean=0, stddev=0.5),
            bias_initializer=tf.constant_initializer(0.1),
            name='fc1'
        )
        # fc2
        layer2 = tf.layers.dense(
            inputs=layer1,
            units=20,
            activation=tf.nn.tanh,  # tanh activation
            kernel_initializer=tf.random_normal_initializer(mean=0, stddev=0.5),
            bias_initializer=tf.constant_initializer(0.1),
            name='fc2'
        )
        # fc3
        all_act = tf.layers.dense(
            inputs=layer2,
            units=self.n_actions,
            activation=None,
            kernel_initializer=tf.random_normal_initializer(mean=0, stddev=0.5),
            bias_initializer=tf.constant_initializer(0.1),
            name='fc3'
        )

        self.all_act_prob = tf.nn.softmax(all_act, name='act_prob')  # use softmax to convert to probability - sum equals to 1

        with tf.name_scope('loss'):
            # to maximize total reward (log_p * R) is to minimize -(log_p * R), and the tf only have minimize(loss)
            neg_log_prob = tf.nn.sparse_softmax_cross_entropy_with_logits(logits=all_act, labels=self.tf_acts)   # this is negative log of chosen action
            # or in this way:
            # neg_log_prob = tf.reduce_sum(-tf.log(self.all_act_prob)*tf.one_hot(self.tf_acts, self.n_actions), axis=1)
            loss = tf.reduce_mean(neg_log_prob * self.tf_vt)  # reward guided loss

        with tf.name_scope('train'):
            self.train_op = tf.train.AdamOptimizer(self.lr).minimize(loss)

    def choose_action(self, observation):
        prob_weights = self.sess.run(self.all_act_prob, feed_dict={self.tf_obs: observation[np.newaxis, :]})
        action = np.random.choice(range(prob_weights.shape[1]), p=prob_weights.ravel())  # select action w.r.t the actions prob
        print('[RL] Choosing action %d with probability %f.' % (action, prob_weights[0][action]))
        return action

    def store_transition(self, s, a, r):
        self.ep_obs.append(s)
        self.ep_as.append(a)
        self.ep_rs.append(r)

    def log_epidose_to_file(self):

        with open('/home/pracsys/catkin_ws/src/rutgers_collab/src/rl_pkg/logs/observations_logs.csv', mode='a') as logfile:
            log_writer = csv.writer(logfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL) 
            # log_writer.writerow(['state', 'action', 'reward'])
            for s, a, r in zip(self.ep_obs, self.ep_as, self.ep_rs):
                log_writer.writerow([s, a, r])

        with open('/home/pracsys/catkin_ws/src/rutgers_collab/src/rl_pkg/logs/training_log.csv', mode='a') as logfile:
            log_writer = csv.writer(logfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL) 
            # log_writer.writerow(['iteration', 'total reward'])
            log_writer.writerow([self.iteration, sum(self.ep_rs)])

        save_path = self.saver.save(self.sess, self.saved_net)

        # Reading would be as following:
        # with open('name.csv') as csv_file:
        #     csv_reader = csv.reader(csv_file, delimiter=',')
        #     for row in csv_reader:
        #         npvector = np.fromstring(row[index][1:-1],dtype=float, sep=' ')


    def learn(self):
        # discount and normalize episode reward
        discounted_ep_rs_norm = self._discount_and_norm_rewards()

        # train on episode
        self.sess.run(self.train_op, feed_dict={
             self.tf_obs: np.vstack(self.ep_obs),  # shape=[None, n_obs]
             self.tf_acts: np.array(self.ep_as),  # shape=[None, ]
             self.tf_vt: discounted_ep_rs_norm,  # shape=[None, ]
        })

        self.log_epidose_to_file()
        self.ep_obs, self.ep_as, self.ep_rs = [], [], []    # empty episode data
        self.iteration += 1
        return discounted_ep_rs_norm

    def _discount_and_norm_rewards(self):
        # discount episode rewards
        discounted_ep_rs = np.zeros_like(self.ep_rs)
        running_add = 0
        for t in reversed(range(0, len(self.ep_rs))):
            running_add = running_add * self.gamma + self.ep_rs[t]
            discounted_ep_rs[t] = running_add

        # normalize episode rewards
        discounted_ep_rs -= np.mean(discounted_ep_rs)
        discounted_ep_rs /= np.std(discounted_ep_rs)
        return discounted_ep_rs



