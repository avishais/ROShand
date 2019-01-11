
import numpy as np
import pickle
import os.path
import matplotlib.pyplot as plt


class transition_experience():
    path = '/home/pracsys/catkin_ws/src/rutgers_collab/src/sim_transition_model/data/'

    def __init__(self, Load=True, discrete = False):

        if discrete:
            self.mode = 'discrete'
        else:
            self.mode = 'cont'
        
        self.file = 'transition_data_' + self.mode + '_v5'
        self.file_name = self.path + self.file + '.obj'

        if Load:
            self.load()
        else:
            self.clear()
        
    def add(self, state, action, next_state, done):
        self.memory += [(state, action, next_state, done)]
        
    def clear(self):
        self.memory = []

    def load(self):
        if os.path.isfile(self.file_name):
            print('Loading data from ' + self.file_name)
            with open(self.file_name, 'rb') as filehandler:
            # filehandler = open(self.file_name, 'r')
                self.memory = pickle.load(filehandler)
            print('Loaded transition data of size %d.'%self.getSize())
        else:
            self.clear()

    def getComponents(self):

        states = np.array([item[0] for item in self.memory])
        actions = np.array([item[1] for item in self.memory])
        next_states = np.array([item[2] for item in self.memory])

        return states, actions, next_states


    def save(self):
        print('Saving data...')
        file_pi = open(self.file_name, 'wb')
        pickle.dump(self.memory, file_pi)
        print('Saved transition data of size %d.'%self.getSize())
        file_pi.close()

    def divide_and_save(self, n = 1000):
        file_training = open('/home/pracsys/catkin_ws/src/rutgers_collab/src/sim_transition_model/data/training_data', 'w')
        pickle.dump(self.memory[n:], file_training)

        file_test = open('/home/pracsys/catkin_ws/src/rutgers_collab/src/sim_transition_model/data/test_data', 'w')
        pickle.dump(self.memory[:n], file_test)

    def getSize(self):
        return len(self.memory)

    def plot_data(self):

        states = [item[0] for item in self.memory]
        done = [item[3] for item in self.memory]
        states = np.array(states)
        failed_states = states[done]

        plt.figure(1)
        ax1 = plt.subplot(121)
        ax1.plot(states[:,0],states[:,1],'-k')
        ax1.plot(states[:,0],states[:,1],'.y')
        ax1.plot(failed_states[:,0],failed_states[:,1],'.r')
        ax1.set(title='Object position')
        
        ax2 = plt.subplot(122)
        ax2.plot(states[:,2],states[:,3],'.k')
        ax2.plot(failed_states[:,2],failed_states[:,3],'.r')
        ax2.set(title='Actuator loads')
        
        # ax3 = plt.subplot(223)
        # ax3.plot(ep_mean_r)
        # ax3.set(title='Mean Episode Rewards')

        # ax4 = plt.subplot(224)
        # ax4.plot(c_losses)
        # ax4.set(title='Q-value losses')

        plt.show()

    def save_to_file(self):

        filen = self.path + self.file + '.db'

        n = self.getSize()

        states = np.array([item[0] for item in self.memory])
        actions = np.array([item[1] for item in self.memory])
        next_states = np.array([item[2] for item in self.memory])
        done = np.array([item[3] for item in self.memory])

        inx = np.where(done)

        # for i in range(len(done)):
        #     if done[i]:
        #         next_states[i] = np.array([-1000.,-1000.,-1000.,-1000.])

        M = np.concatenate((states, actions, next_states), axis=1)
        M = np.delete(M, inx, 0)

        np.savetxt(filen, M, delimiter=' ')

