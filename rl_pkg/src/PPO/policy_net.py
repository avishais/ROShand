import gym
import numpy as np
import tensorflow as tf


class Policy_net:
    def __init__(self, name, n_inputs, n_outputs, temp=0.1):
        """
        :param name: string
        :param env: gym env
        :param temp: temperature of boltzmann distribution
        """

        ob_space = (n_inputs,)
        act_space = n_outputs

        with tf.variable_scope(name):
            self.obs = tf.placeholder(dtype=tf.float32, shape=[None] + list(ob_space), name='obs')

            with tf.variable_scope('policy_net'):
                layer_1 = tf.layers.dense(inputs=self.obs, units=20, activation=tf.tanh, kernel_initializer=tf.random_normal_initializer(mean=0, stddev=0.5), bias_initializer=tf.constant_initializer(0.1))
                layer_2 = tf.layers.dense(inputs=layer_1, units=20, activation=tf.tanh, kernel_initializer=tf.random_normal_initializer(mean=0, stddev=0.5), bias_initializer=tf.constant_initializer(0.1))
                layer_3 = tf.layers.dense(inputs=layer_2, units=act_space, activation=tf.tanh, kernel_initializer=tf.random_normal_initializer(mean=0, stddev=0.5), bias_initializer=tf.constant_initializer(0.1))
                self.act_probs = tf.layers.dense(inputs=tf.divide(layer_3, temp), units=act_space, activation=tf.nn.softmax)

            with tf.variable_scope('value_net'):
                layer_1 = tf.layers.dense(inputs=self.obs, units=20, activation=tf.tanh)
                layer_2 = tf.layers.dense(inputs=layer_1, units=20, activation=tf.tanh)
                self.v_preds = tf.layers.dense(inputs=layer_2, units=1, activation=None)

            self.act_stochastic = tf.multinomial(tf.log(self.act_probs), num_samples=1)
            self.act_stochastic = tf.reshape(self.act_stochastic, shape=[-1])

            self.act_deterministic = tf.argmax(self.act_probs, axis=1)

            self.scope = tf.get_variable_scope().name

    def act(self, obs, stochastic=True):
        # p = tf.get_default_session().run(self.act_probs, feed_dict={self.obs: obs})
        # print("Probs: ", p, sum(p[0]))
        if stochastic:
            return tf.get_default_session().run([self.act_stochastic, self.v_preds], feed_dict={self.obs: obs})
        else:
            return tf.get_default_session().run([self.act_deterministic, self.v_preds], feed_dict={self.obs: obs})

    def get_action_prob(self, obs):
        return tf.get_default_session().run(self.act_probs, feed_dict={self.obs: obs})

    def get_variables(self):
        return tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, self.scope)

    def get_trainable_variables(self):
        return tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, self.scope)

