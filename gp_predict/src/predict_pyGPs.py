#!/usr/bin/env python

import numpy as np
from sklearn.neighbors import KDTree #pip install -U scikit-learn
import pyGPs
import time
from scipy.io import loadmat

class predict:
    K = 100 # Number of NN
    mode = 5

    state_action_dim = 4
    state_dim = 2

    Xtrain = np.array([])
    Ytrain = np.array([])

    x_max_X = np.array([])
    x_min_X = np.array([])
    x_max_Y = np.array([])
    x_min_Y = np.array([])

    kdt = []

    def __init__(self, mode = 5):

        print('[gp_predict pyGPs] Loading training data...')
        self.mode = mode
        Q = loadmat('/home/pracsys/Documents/workspace/adaptive_hand_model/data/Ca_25_' + str(mode) + '.mat')
        Qtrain = Q['Xtraining']
        print('[gp_predict pyGPs] Loaded training data of ' + str(Qtrain.shape[0]) + ' points.')

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

        # Load kd-tree
        print("[gp_predict pyGPs] Constructing kd-tree...")
        # Xtrain_nn = self.Xtrain_# * W
        self.kdt = KDTree(self.Xtrain, leaf_size=10, metric='euclidean')
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
        idx = self.kdt.query(sa, k=self.K, return_distance=False)
        X_nn = self.Xtrain[idx,:].reshape(self.K, self.state_action_dim)
        Y_nn = self.Ytrain[idx,:].reshape(self.K, self.state_dim)

        mu = np.zeros(self.state_dim)
        sigma = np.zeros(self.state_dim)
        for dim in range(self.state_dim):
            model = pyGPs.GPR_FITC()      # specify model (GP regression)

            #  Set number of latent inducing points (Andersson et al. 2015)
            num_u = np.fix(15)
            u = np.tile(np.linspace(0,1,num_u).T, (1, self.state_action_dim))
            u = np.reshape(u,(int(num_u), self.state_action_dim))

            m = pyGPs.mean.Linear( D=X_nn.shape[1] )
            k = pyGPs.cov.RBF(log_ell=5., log_sigma=-5)
            model.setPrior(mean=m, kernel=k, inducing_points=u) 

            # Optimize
            model.setData(X_nn, Y_nn[:,dim]) # fit default model (mean zero & rbf kernel) with data
            model.getPosterior()
            model.optimize(X_nn, Y_nn[:,dim])     # optimize hyperparamters (default optimizer: single run minimize)

            # Predict
            model.predict(sa.reshape(1, self.state_action_dim))
            mu[dim] = model.ym
            sigma[dim] = np.sqrt(model.ys2)

        return mu, sigma

    def propagate(self, sa):
        mu, sigma = self.predict(sa)
        # s_next = np.random.normal(mu, sigma, self.state_dim)
        
        return mu#s_next

    def countNN(self, sa):
        idx = self.kdt.query_radius(sa, r=0.05)
        return len(idx[0])

