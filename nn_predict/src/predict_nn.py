""" 
Author: Avishai Sintov
"""
from __future__ import division, print_function, absolute_import

from nn_functions import * # My utility functions

import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from scipy.io import loadmat
import time
from sklearn.neighbors import KDTree #pip install -U scikit-learn

class predict_nn:

    mode = 8
    state_action_dim = 8
    state_dim = 6

    Xtrain = np.array([])
    Xtrain_nn = np.array([])
    kdt = []
    W = []
       
    def __init__(self):

        ####### Load training data for failure check
        print('[predict_nn] Loading training data...')
        Q = loadmat('/home/pracsys/Documents/workspace/adaptive_hand_model/data/Cc_20_' + str(self.mode) + '.mat')
        Qtrain = Q['Xtraining']
        print('[predict_nn Gpy] Loaded training data of ' + str(Qtrain.shape[0]) + ' points in feature conf. ' + str(self.mode) + '.')

        self.W = np.array([1, 1, 1, 1, 1, 1, 10, 10])
        self.W = self.W.reshape((self.W.shape[0],))
        self.Xtrain = Qtrain[:,0:self.state_action_dim]

        self.x_max_X = np.max(self.Xtrain, axis=0)
        self.x_min_X = np.min(self.Xtrain, axis=0)
        self.Xtrain = self.normalize(self.Xtrain)

        print("[predict_nn] Constructing kd-tree...")
        self.Xtrain_nn = self.Xtrain# * self.W
        self.kdt = KDTree(self.Xtrain_nn, leaf_size=10, metric='euclidean')
        print("[predict_nn] kd-tree constructed. Ready to predict!")


        ######## Load Neural Network
        model_file = "/home/pracsys/Documents/workspace/adaptive_hand_model/nn/models/cc_8.ckpt"

        # Network Parameters
        self.num_input = self.state_action_dim
        self.num_output = self.state_dim
        self.hidden_layers = [72]*2
        self.activation = 2

        self.x_mu = np.array([ 7.53983535e+01, -3.38066395e+02,  5.28607692e-01,  4.26463315e-01, 1.02009669e+02, -9.61020193e+01,  9.18136446e-03,  8.20907553e-03, -4.26041625e-03,  3.07213955e-02,  3.01654645e-05,  2.67301021e-05, 7.55110990e-03, -7.53022741e-03])
        self.x_sigma = np.array([1.20656722e+02, 5.48090069e+01, 6.56873644e-02, 6.43842025e-02, 9.15623420e+01, 8.60852468e+01, 5.92933601e-02, 5.94357727e-02, 3.76585272e-01, 1.58610485e-01, 3.96773340e-04, 1.80385971e-03, 5.84777356e+00, 5.63405418e+00])

        # tf Graph input 
        self.X = tf.placeholder("float", [None, self.num_input])
        self.Y = tf.placeholder("float", [None, self.num_output])

        # Store layers weight & bias
        self.weights, self.biases = wNb(self.num_input, self.hidden_layers, self.num_output)

        self.prediction = neural_net(self.X, self.weights, self.biases, self.activation)

        # cost = tf.reduce_mean(0.5*tf.pow(prediction - Y, 2))#/(2*n)

        self.sess = tf.Session()

        # Restore variables from disk.
        saver = tf.train.Saver()
        saver.restore(self.sess, model_file)

    def normalize(self, data):
        for i in range(data.shape[1]):
            data[:,i] = (data[:,i]-self.x_min_X[i])/(self.x_max_X[i]-self.x_min_X[i])
        return data

    def predict(self, sa):
        s = np.copy(sa[:,:self.num_output])#.reshape(self.num_input, 1)
        
        sa = normzG(sa.reshape(1,-1), self.x_mu[:self.num_input], self.x_sigma[:self.num_input])
        ds = self.sess.run(self.prediction, {self.X: sa.reshape(1,self.num_input)})
        ds = denormzG(ds, self.x_mu[self.num_input:], self.x_sigma[self.num_input:])
        
        s_next = s + ds
        return s_next

    def countNN(self, sa):
        idx = self.kdt.query_radius(sa, r=np.sqrt(0.15))
        return len(idx[0])

        
# if __name__ == "__main__":
#     NN = predict_nn()

#     sa = np.array([67.7478, -409.5858 ,   0.4772,    0.3560,   61.0000,         0, 0.0600,   -0.0600])
#     s_next = NN.predict(sa)
#     # print(s_next)





