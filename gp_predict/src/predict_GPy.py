#!/usr/bin/env python

import numpy as np
from sklearn.neighbors import KDTree #pip install -U scikit-learn
import GPy
import time
from scipy.io import loadmat
from matplotlib import pyplot as plt

class predict:
    K = 100 # Number of NN
    mode = 5

    state_action_dim = 4
    state_dim = 2

    Xtrain = np.array([])
    Ytrain = np.array([])
    Xtrain_nn = np.array([])
    W = np.array([])

    x_max_X = np.array([])
    x_min_X = np.array([])
    x_max_Y = np.array([])
    x_min_Y = np.array([])

    kdt = []

    def __init__(self, mode = 5):

        print('[gp_predict Gpy] Loading training data...')
        self.mode = mode
        Q = loadmat('/home/pracsys/Documents/workspace/adaptive_hand_model/data/Cb_20_' + str(mode) + '.mat')
        Qtrain = Q['Xtraining']
        print('[gp_predict Gpy] Loaded training data of ' + str(Qtrain.shape[0]) + ' points in feature conf. ' + str(mode) + '.')

        if self.mode==1:
            self.state_action_dim = 4 
            self.state_dim = 2
        if self.mode==2:
            self.state_action_dim = 8 
            self.state_dim = 6
        if self.mode==3:
            self.state_action_dim = 12 
            self.state_dim = 10
        if self.mode==4:
            self.state_action_dim = 6 
            self.state_dim = 4
        if self.mode==5:
            self.state_action_dim = 6 
            self.state_dim = 4
        if self.mode==6:
            self.state_action_dim = 14 
            self.state_dim = 12
        if self.mode==7:
            self.state_action_dim = 16
            self.state_dim = 14
        if self.mode==8:
            self.state_action_dim = 8
            self.state_dim = 6

        self.Xtrain = Qtrain[:,0:self.state_action_dim]
        self.Ytrain = Qtrain[:,self.state_action_dim:]

        # Normalization parameters
        self.x_max_X = np.max(self.Xtrain, axis=0)
        self.x_min_X = np.min(self.Xtrain, axis=0)
        self.x_max_Y = np.max(self.Ytrain, axis=0)
        self.x_min_Y = np.min(self.Ytrain, axis=0)
        for i in range(self.state_dim):
            tmp = np.max([self.x_max_X[i], self.x_max_Y[i]])
            self.x_max_X[i] = tmp
            self.x_max_Y[i] = tmp
            tmp = np.min([self.x_min_X[i], self.x_min_Y[i]])
            self.x_min_X[i] = tmp
            self.x_min_Y[i] = tmp
        self.Xtrain_ = self.normalize(self.Xtrain, 1)
        self.Ytrain_ = self.normalize(self.Ytrain, 2)

        # Weight matrix for the kd-tree
        self.W = np.concatenate( ( np.array([np.sqrt(1000.), np.sqrt(1000.)]).reshape(1,2), np.ones((1,self.state_dim)) ), axis=1 ).T
        self.W = self.W.reshape((self.W.shape[0],))

        # Load kd-tree
        print("[gp_predict Gpy] Constructing kd-tree...")
        self.Xtrain_nn = self.Xtrain * self.W
        self.kdt = KDTree(self.Xtrain_nn, leaf_size=10, metric='euclidean')
        print("[gp_predict pyGPs] kd-tree constructed. Ready to predict!")

    def normalize(self, data, c=1):
        if c == 1:
            for i in range(data.shape[1]):
                data[:,i] = (data[:,i]-self.x_min_X[i])/(self.x_max_X[i]-self.x_min_X[i])
        else:
            if c == 2:
                for i in range(data.shape[1]):
                    data[:,i] = (data[:,i]-self.x_min_Y[i])/(self.x_max_Y[i]-self.x_min_Y[i])

        return data

    def denormalize(self, data, c=1):
        if c == 1:
            for i in range(data.shape[1]):
                data[:,i] = data[:,i]*(self.x_max_X[i]-self.x_min_X[i]) + self.x_min_X[i]
        else:
            if c == 2:
                for i in range(data.shape[1]):
                    data[:,i] = data[:,i]*(self.x_max_Y[i]-self.x_min_Y[i]) + -self.x_min_Y[i]

        return data

    def predict(self, sa):
        idx = self.kdt.query(sa*self.W, k=self.K, return_distance=False)
        X_nn = self.Xtrain[idx,:].reshape(self.K, self.state_action_dim)
        Y_nn = self.Ytrain[idx,:].reshape(self.K, self.state_dim)

        # Plot NN
        # st = sa[0]#.reshape(1,self.state_action_dim)
        # fig = plt.figure(1)
        # plt.plot(X_nn[:,0],X_nn[:,1],'.y')
        # plt.plot(st[0],st[1],'ob')
        # plt.show()

        mu = np.zeros(self.state_dim)
        sigma = np.zeros(self.state_dim)
        for dim in range(self.state_dim):
            kernel = GPy.kern.RBF(input_dim=self.state_action_dim, variance=1., lengthscale=1.)
            m = GPy.models.GPRegression(X_nn,Y_nn[:,dim].reshape(-1,1),kernel)

            m.optimize(messages=False)
            # m.optimize_restarts(num_restarts = 10)

            mu[dim], sigma[dim] = m.predict(sa.reshape(1,self.state_action_dim))

        plt.plot(mu[0],mu[1],'sr')
        return mu, sigma

    def propagate(self, sa):
        mu, sigma = self.predict(sa)
        # s_next = np.random.normal(mu, sigma, self.state_dim)
        
        return mu#s_next

    def countNN(self, sa):
        idx = self.kdt.query_radius(sa, r=0.05)
        return len(idx[0])





