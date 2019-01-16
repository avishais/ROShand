#!/usr/bin/env python

import numpy as np
import pickle
from sklearn.neighbors import KDTree #pip install -U scikit-learn
from sklearn import svm
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt

take_test_data = True
gen_new_data = 1

version = 5

class SVM_failure():

    r = 0.1

    def __init__(self, discrete = True):

        self.mode = 'discrete' if discrete else 'cont'

        self.load_data(gen_new_data)

        print 'All set!'

    def load_data(self, gen=True):

        path = '/home/pracsys/catkin_ws/src/rutgers_collab/src/sim_transition_model/data/'
        # path = '../data/'

        if gen:
            file_name = path + 'transition_data_' + self.mode + '_v' + str(version) + '.obj'
            print('Loading data from ' + file_name)
            with open(file_name, 'rb') as filehandler:
                self.memory = pickle.load(filehandler)
            print('Loaded transition data of size %d.'%len(self.memory))

            self.states = np.array([item[0] for item in self.memory])
            self.actions = np.array([item[1] for item in self.memory])
            self.done = np.array([item[3] for item in self.memory])

            if file_name == path + 'transition_data_' + self.mode + '_v3.obj':
                self.states = self.states[:,:4] # for transition_data_discrete_v3 data set (contains velocity)

            # Process data
            self.SA = np.concatenate((self.states, self.actions), axis=1)

            # Sparser
            T = np.where(self.done)[0]
            inx_fail = T
            print "Number of failed states " + str(T.shape[0])
            T = np.where(np.logical_not(self.done))[0]
            inx_suc = T[np.random.choice(T.shape[0], 1000, replace=False)]
            self.SA = np.concatenate((self.SA[inx_fail], self.SA[inx_suc]), axis=0)
            self.done = np.concatenate((self.done[inx_fail], self.done[inx_suc]), axis=0)

            with open(path + 'svm_data_' + self.mode + '_v' + str(version) + '.obj', 'wb') as f: 
                pickle.dump([self.SA, self.done], f)
            print('Saved svm data.')
        else:
            print('Loading data from ' + 'svm_data_' + self.mode + '.obj')
            with open(path + 'svm_data_' + self.mode + '_v' + str(version) + '.obj', 'rb') as f: 
                self.SA, self.done = pickle.load(f)
            print('Loaded svm data.')
            

        # Normalize
        scaler = StandardScaler()
        self.SA = scaler.fit_transform(self.SA)
        self.x_mean = scaler.mean_
        self.x_std = scaler.scale_

        # Test data
        if take_test_data:
            ni = 40
            T = np.where(self.done)[0]
            inx_fail = T[np.random.choice(T.shape[0], ni, replace=False)]
            T = np.where(np.logical_not(self.done))[0]
            inx_suc = T[np.random.choice(T.shape[0], ni, replace=False)]
            self.SA_test = np.concatenate((self.SA[inx_fail], self.SA[inx_suc]), axis=0)
            self.done_test = np.concatenate((self.done[inx_fail], self.done[inx_suc]), axis=0)

            self.SA = np.delete(self.SA, inx_fail, axis=0)
            self.SA = np.delete(self.SA, inx_suc, axis=0)
            self.done = np.delete(self.done, inx_fail, axis=0)
            self.done = np.delete(self.done, inx_suc, axis=0)

        print 'Fitting SVM...'
        self.clf = svm.SVC( probability=True, class_weight='balanced', C=1.0 )
        self.clf.fit( list(self.SA), 1*self.done )
        print 'SVM ready with %d classes: '%len(self.clf.classes_) + str(self.clf.classes_)

    def probability(self, sa):

        # Normalize
        if not take_test_data:
            sa = (sa - self.x_mean) / self.x_std

        p = self.clf.predict_proba(sa)[0]

        return p, self.clf.predict(sa)


if __name__ == '__main__':
    
    K = SVM_failure(True)

    if take_test_data:

        if take_test_data:
            SA_test = K.SA_test
            s = 0
            s_suc = 0; c_suc = 0
            s_fail = 0; c_fail = 0
            for i in range(SA_test.shape[0]):
                # print SA_test[i]
                p, fail = K.probability(SA_test[i].reshape(1,-1))
                fail = p[1]>0.5
                print p, K.done_test[i], fail
                s += 1 if fail == K.done_test[i] else 0
                if K.done_test[i]:
                    c_fail += 1
                    s_fail += 1 if fail else 0
                else:
                    c_suc += 1
                    s_suc += 1 if not fail else 0
            print 'Success rate: ' + str(float(s)/SA_test.shape[0]*100)
            print 'Drop prediction accuracy: ' + str(float(s_fail)/c_fail*100)
            print 'Success prediction accuracy: ' + str(float(s_suc)/c_suc*100)

    else:

        s_max = np.array([93.31538391, 143.04187012,  66.03091431,  79.83042908])
        s_min = np.array([-87.74127197,  -5.84844971,   2.18996787,   0.47996914])
        
        N = 100
        sx = np.linspace(s_min[0], s_max[0], N)
        sy = np.linspace(s_min[1], s_max[1], N)
        Sx, Sy = np.meshgrid(sx, sy)

        load = np.array([30.0, 30.0])
        action = np.array([1.,-1.]) # [1,-1]-right, [-1,1]-left
        A = np.array([0,-1,1])

        C = np.zeros((N,N))
        for i in range(N):
            for j in range(N):
                pos = np.array([Sx[i,j], Sy[i,j]])
                sa = np.concatenate((pos, load, action), axis = 0)
                p, fail = K.probability(sa.reshape(1,-1))
                # print sa, p, fail[0]==1
                C[i,j] = p[1]

        h = plt.contourf(sx,sy,C)
        plt.colorbar(h)
        plt.show()



    

    





