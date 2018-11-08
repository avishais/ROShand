""" 
Author: Avishai Sintov
"""
from __future__ import division, print_function, absolute_import

from nn_functions import * # My utility functions
from transition_experience import *

import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import time

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("-r", help="Retrain existing model", action="store_true")
parser.add_argument("-p", help="Plot trained models", action="store_true")
args = parser.parse_args()
if args.r and args.p:
    training = True
    retrain = True
if args.r:
    training = True
    retrain = True
elif args.p:
    training = False
    retrain = False
else:
    training = True
    retrain = False

DropOut = False
Regularization = False

print('Loading training data...')

mode = 8

texp = transition_experience()
prev_states, actions, next_states = texp.getComponents()
# Xtest = Q['Xtest1']['data'][0][0]

num_input = 6 
num_output = 4

n_test = 750 #int(np.round(0.1*prev_states.shape[0]))
n = prev_states.shape[0]-n_test

# Network Parameters
hidden_layers = [72]*3
activation = 2

X = np.concatenate((prev_states, actions, next_states-prev_states), axis=1)

x_max = np.max(X, axis=0)
x_min = np.min(X, axis=0)
x_mu = np.mean(X, axis=0)
x_sigma = np.std(X, axis=0)
X = normzG(X, x_mu, x_sigma)

x_train = X[n_test:,0:num_input]
y_train = X[n_test:,num_input:]
x_test = X[:n_test,0:num_input]
y_test = X[:n_test,num_input:]

# Training Parameters
learning_rate =  0.005
num_steps = int(5e5)
batch_size = 250
display_step = 100

# tf Graph input 
X = tf.placeholder("float", [None, num_input])
Y = tf.placeholder("float", [None, num_output])

# Store layers weight & bias
weights, biases = wNb(num_input, hidden_layers, num_output)

# Construct model
keep_prob_input = tf.placeholder(tf.float32)
keep_prob = tf.placeholder(tf.float32)
if not DropOut:
    prediction = neural_net(X, weights, biases, activation)
else:
    X_drop = tf.nn.dropout(X, keep_prob=keep_prob_input)
    prediction = neural_net_dropout(X, weights, biases, keep_prob, activation)


# Define loss 
cost = tf.reduce_mean(0.5*tf.pow(prediction - Y, 2))#/(2*n)
# cost = tf.reduce_mean(np.absolute(y_true - y_pred))
# cost = tf.reduce_sum(tf.square(prediction - Y))

# L2 Regularization
if Regularization:
    beta = 0.01
    regularizer = computeReg(weights)
    cost = cost + beta * regularizer

# Define optimizer
# optimizer = tf.train.AdamOptimizer(learning_rate)
# optimizer = tf.train.GradientDescentOptimizer(learning_rate)
optimizer = tf.train.AdagradOptimizer(learning_rate)
train_op = optimizer.minimize(cost)

# Initialize the variables (i.e. assign their default value)
init = tf.global_variables_initializer()

# Add ops to save and restore all the variables.
saver = tf.train.Saver()

load_from = 'nn_model.ckpt'
save_to = 'nn_model.ckpt'

# Start Training
# Start a new TF session
COSTS = []	# for plotting
STEPS = []	# for plotting
start = time.time()
with tf.Session() as sess:

    if training:
    
        if  not retrain:
            # Run the initializer
            sess.run(init)
        else:
            # Restore variables from disk.
            saver.restore(sess, "../data/" + load_from)                
            # print("Loaded saved model: %s" % "./models/" + load_from)

        # Training
        for i in range(1, num_steps+1):
            # Get the next batch 
            batch_x, batch_y = next_batch(batch_size, x_train, y_train)

            # Run optimization op (backprop) and cost op (to get loss value)
            if not DropOut:
                _, c = sess.run([train_op, cost], feed_dict={X: batch_x, Y: batch_y})
            else:
                _, c = sess.run([train_op, cost], feed_dict={X: batch_x, Y: batch_y, keep_prob_input: 0.5, keep_prob: 0.5})
            # Display logs per step
            if i % display_step == 0 or i == 1:
                print('Step %i: Minibatch Loss: %f' % (i, c))
                save_path = saver.save(sess, "../data/" + save_to)
                COSTS.append(c)
                STEPS.append(i)

        print("Optimization Finished!")

        # Save the variables to disk.
        save_path = saver.save(sess, "../data/" + save_to)
        print("Model saved in path: %s" % save_path)

        # Plot cost convergence
        plt.figure(4)
        plt.semilogy(STEPS, COSTS, 'k-')
        plt.xlabel('Step')
        plt.ylabel('Cost')
        plt.ylim([0, np.max(COSTS)])
        plt.grid(True)
    else:
        # Restore variables from disk.

        saver.restore(sess, "../data/" + load_from)

    # Testing
    # Calculate cost for training data
    y_train_pred = sess.run(prediction, {X: x_train, keep_prob_input: 1.0, keep_prob: 1.0})
    training_cost = sess.run(cost, feed_dict={X: x_train, Y: y_train, keep_prob_input: 1.0, keep_prob: 1.0})
    print("Training cost:", training_cost)

    y_test_pred = sess.run(prediction, {X: x_test, keep_prob_input: 1.0, keep_prob: 1.0})
    testing_cost = sess.run(cost, feed_dict={X: x_test, Y: y_test, keep_prob_input: 1.0, keep_prob: 1.0})
    print("Testing cost=", testing_cost)

    # j = 100 # np.random.random_integers(1, x_train.shape[0]) %np.array([-0.1177 ,   0.0641  ,  0.9617  ,  0.9387])#
    # f = np.copy(x_test[j, :]) #.reshape(1,num_input)
    # f = f.reshape(1,num_input)
    # y = sess.run(prediction, {X: f, keep_prob_input: 1.0, keep_prob: 1.0})
    # print("Testing point: ", f)
    # print("Point ", j, ": ", y, y_test[j,:])
    # # xo = denormz(x_test[j, 0:2], x_max, x_min)
    # # yo = denormz(y, x_max[4:], x_min[4:])
    # f = f.reshape(num_input, 1)
    # xo = denormzG(f[0:2], x_mu, x_sigma)
    # yo = denormzG(y, x_mu[num_input:], x_sigma[num_input:])
    # print("y ", yo)
    # yr = denormzG(y_test[j,:], x_mu[num_input:], x_sigma[num_input:])
    # # print(x_mu, x_sigma)
    # print("State: ", xo.reshape(1,2))
    # print("Predicted next state: ", xo.reshape(1,2) + yo[:,0:2])
    # print("Real next step: ", xo.reshape(1,2) + yr[0:2])

    # Open-loop
    # plt.figure(2)
    # st = 100
    # en = 110
    # S = x_test[st:en,:]
    # n = S.shape[0]
    # sa = np.copy(S[0,:]).reshape(1,num_input)
    # sa_dn = denormzG(np.copy(sa).reshape(num_input, 1)[0:2], x_mu, x_sigma)
    # for i in range(st+1, en):
    #     print(sa_dn.reshape(1,2))
        
    #     ds_predicted = sess.run(prediction, {X: sa, keep_prob_input: 1.0, keep_prob: 1.0})
    #     ds_predicted_dn = denormzG(np.copy(ds_predicted), x_mu[num_input:], x_sigma[num_input:])

    #     s_next_predicted_dn = sa_dn.reshape(1,2) + ds_predicted_dn[:,0:2]
    #     s_next_predicted = sa.reshape(1,6)[:,:4] + ds_predicted[:,0:4]

    #     plt.plot([sa_dn[0], s_next_predicted_dn[0][0]], [sa_dn[1], s_next_predicted_dn[0][1]], '-b')

    #     sa = np.concatenate((s_next_predicted, x_test[i,4:6].reshape(1,2)), axis=1)
    #     sa_dn = s_next_predicted_dn[0]

    # S = np.array([denormzG(sa.reshape(num_input, 1)[0:2], x_mu, x_sigma).reshape(1,2) for sa in S]).reshape(n,2)
    # plt.plot(S[:,0], S[:,1], 'o:k')

    export_net(weights, biases, x_mu, x_sigma, activation, sess, '../data/net_transition.netxt')

plt.show()


