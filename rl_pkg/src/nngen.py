#!/usr/bin/env python

import tensorflow as tf
import numpy as np

def normz(x, x_max, x_min):

    for i in range(x.shape[1]):
        x[:,i] = (x[:,i]-x_min[i])/(x_max[i]-x_min[i])
    
    # x = (x-x_min)/(x_max-x_min)

    return x

def denormz(x, x_max, x_min):
    x = x.reshape(2,)
    for i in range(x.shape[0]):
        x[i] = x[i]*(x_max[i]-x_min[i]) + x_min[i]

    # x = x*(x_max-x_min) + x_min
    
    return x

def normzG(x, mu, sigma):
    
    for i in range(x.shape[1]):
        x[:,i] = (x[:,i]-mu[i])/sigma[i]
    
    # x = (x-x_min)/(x_max-x_min)

    return x

def denormzG(x, mu, sigma):
    # x = x.reshape(2,)
    for i in range(x.shape[0]):
        x[i] = x[i]*sigma[i] + mu[i]

    # x = x*(x_max-x_min) + x_min
    
    return x

# -----------------------------------------------------------------------

def next_batch(num, data, labels):
    '''
    Return a total of `num` random samples. 
    Similar to mnist.train.next_batch(num)
    '''
    idx = np.arange(0 , len(data))
    np.random.shuffle(idx)
    idx = idx[:num]
    data_shuffle = [data[ i] for i in idx]
    labels_shuffle = [labels[ i] for i in idx]

    return np.asarray(data_shuffle), np.asarray(labels_shuffle)

# Building the net
def neural_net(n_inputs, n_outputs):
    initializer = tf.contrib.layers.variance_scaling_initializer()

    X = tf.placeholder(tf.float32, shape=[None, n_inputs])
    Y = tf.placeholder(tf.float32, shape=[None, n_outputs])

    hidden = tf.layers.dense(X, n_inputs, activation=tf.nn.elu, kernel_initializer=initializer, bias_initializer=tf.zeros_initializer())
    logits = tf.layers.dense(hidden, n_outputs)
    outputs = tf.nn.sigmoid(logits) # probability of action 0 (left)
    
    return outputs, X, Y