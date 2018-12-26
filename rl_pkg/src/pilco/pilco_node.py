#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt

from pilco import PILCO
from controllers import RbfController, LinearController
from rewards import ExponentialReward, ExponentialRewardPosition

from std_msgs.msg import Float64MultiArray, Float32MultiArray, Int16, String
from std_srvs.srv import SetBool, Empty, EmptyResponse
from rl_pkg.srv import TargetAngles, IsDropped, observation

# np.random.seed(0)

# state_dim = 4
# control_dim = 2

# # There is a tensorflow error if this are in the class
# # controller = RbfController(state_dim=state_dim, control_dim=control_dim, num_basis_functions=5)


class pilco_node():
    goal = np.array([37.0,  106.0,    9.6664,   10.5664 ])
    max_steps = 50
    stop_distance = 0.08
    state_max = np.array([ 90.0164,  140.6792,   67.6822,   66.9443])
    state_min = np.array([-92.6794,   44.1619,    0.4516,    0.9372])

    def __init__(self):
        rospy.init_node('pilco_node', anonymous=True)
        self.rate = rospy.Rate(15)

        rospy.Subscriber('/RL/gripper_status', String, self.callbackGripperStatus)

        self.reset_srv = rospy.ServiceProxy('/RL/ResetGripper', Empty)
        self.move_srv = rospy.ServiceProxy('/RL/MoveGripper', TargetAngles)
        self.drop_srv = rospy.ServiceProxy('/RL/IsObjDropped', IsDropped)
        self.obs_srv = rospy.ServiceProxy('/RL/observation', observation)
        # self.plot_srv = rospy.ServiceProxy('/plot/plot', Empty)
        # self.clear_srv = rospy.ServiceProxy('/plot/clear', Empty)

        pub_goal = rospy.Publisher('/RL/goal', Float32MultiArray, queue_size=10)
        self.pub_act = rospy.Publisher('/RL/action', Float32MultiArray, queue_size=10)
        msg = Float32MultiArray()

        msg.data = self.goal
        # Initial random rollouts to generate a dataset
        X,Y = self.rollout(policy=self.random_policy, steps=self.max_steps)
        for i in range(1,15):
            X_, Y_ = self.rollout(policy=self.random_policy, steps=self.max_steps)
            X = np.vstack((X, X_))
            Y = np.vstack((Y, Y_))

        data_size = X.shape[0]
        self.state_dim = Y.shape[1]
        control_dim = X.shape[1] - self.state_dim
        controller = RbfController(state_dim=self.state_dim, control_dim=control_dim, num_basis_functions=5)
        # controller = LinearController(state_dim=self.state_dim, control_dim=control_dim)

        # X = self.normz(X); Y = self.normz(Y)
        self.ngoal = np.zeros(self.state_dim)
        for i in range(self.state_dim):
            self.ngoal[i] = (self.goal[i]-self.state_min[i])/(self.state_max[i]-self.state_min[i])

        # pilco = PILCO(X, Y, controller=controller, horizon=self.max_steps)
        # Example of user provided reward function, setting a custom target state
        # R = ExponentialReward(state_dim=self.state_dim, t=self.ngoal)
        R = ExponentialRewardPosition(state_dim=self.state_dim, t=self.ngoal) # Only position
        self.pilco = PILCO(X, Y, controller=controller, horizon=self.max_steps, reward=R, num_induced_points=int(data_size/10))

        # Example of fixing a parameter, optional, for a linear controller only
        #pilco.controller.b = np.array([[0.0]])
        #pilco.controller.b.trainable = False

        Iter = 0
        success_count = 0
        while not rospy.is_shutdown():
            pub_goal.publish(msg)

            Iter += 1
            self.pilco.optimize()
            # import pdb; pdb.set_trace()
            X_new, Y_new = self.rollout(policy=self.pilco_policy, steps=self.max_steps)
            # X_new = self.normz(X_new); Y_new = self.normz(Y_new)
            # Update dataset
            X = np.vstack((X, X_new)); Y = np.vstack((Y, Y_new))
            self.pilco.mgpr.set_XY(X, Y)

            cur = np.array(self.obs_srv().state)
            d = np.linalg.norm((X_new[-1,:2]+Y_new[-1,:2])-self.ngoal[:2])
            # print('[pilco_node] Iteration ' + str(Iter) + ', reached:' + str(X_new[-1,:self.state_dim]+Y_new[-1,:]) + ', distance: ' + str( np.linalg.norm((X_new[-1,:self.state_dim]+Y_new[-1,:])-self.ngoal) ))
            print('[pilco_node] Goal: ' + str(self.goal) + ', normalized goal: ' + str(self.ngoal) + ', current position: ' + str(cur))
            print('[pilco_node] Iteration ' + str(Iter) + ', reached:' + str(X_new[-1,:2]+Y_new[-1,:2]) + ', pos. distance: ' + str( d ))
            # if np.linalg.norm(self.ngoal-X_new[-1,:self.state_dim]) < self.stop_distance:
            if d < self.stop_distance:
                print('[pilco_node] Goal reached after %d iterations, %d/3 trial.!'%(Iter, success_count))
                success_count += 1
                if success_count >= 3:
                    break
            else:
                success_count = 0
            self.rate.sleep()

        print('[pilco_node] Found solution.')
        X, Y = self.rollout(policy=self.pilco_policy, steps=self.max_steps, normz=False)
        plt.figure()
        plt.plot(X[:,0], X[:,1],'-k')
        plt.plot(X[0,0], X[0,1],'or')
        plt.plot(self.goal[0], self.goal[1],'og')
        xend = X[-1,:self.state_dim] + Y[-1,:]
        plt.plot(xend[0], xend[1],'ob')
        plt.axis('equal')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.show()
            

    def rollout(self, policy, steps, normz=True):
        X = []; Y = []
        print('[pilco_node] Rollout...')        

        # Reset system
        # self.clear_srv()
        self.reset_srv()
        while not self.gripper_closed:
            self.rate.sleep()

        # Get observation
        x = np.array(self.obs_srv().state)
        state_dim = len(x)

        for i in range(steps):
            # print('Step ' + str(i) + '...')

            xu = np.zeros(4)
            for i in range(state_dim):
                xu[i] = (x[i]-self.state_min[i])/(self.state_max[i]-self.state_min[i])

            # Choose action
            u = policy(xu)
            # print('u: ', u)

            u[2] = (u[2]+1)/2

            msg = Float32MultiArray()
            msg.data = u
            self.pub_act.publish(msg)

            # Act
            for _ in range(int(u[2]*150)+1):
                suc = self.move_srv(u[:2]).success
                rospy.sleep(0.05)
                self.rate.sleep()
                if not suc:
                    break

            fail = self.drop_srv().dropped # Check if dropped - end of episode

            if suc and not fail:
                # Get observation
                x_new = np.array(self.obs_srv().state)
            else:
                # End episode if overload or angle limits reached
                rospy.logerr('[pilco_node] Failed to move. Episode declared failed.')
                break
            
            xu = np.hstack((x, u))
            dx = x_new - x
            if normz:
                dx = np.zeros(state_dim)
                for i in range(state_dim):
                    xu[i] = (xu[i]-self.state_min[i])/(self.state_max[i]-self.state_min[i])
                    dx[i] = (x_new[i]-self.state_min[i])/(self.state_max[i]-self.state_min[i]) - xu[i] 

            X.append(xu)
            Y.append(dx)
            x = x_new

            self.rate.sleep()

        # self.plot_srv()
        return np.stack(X), np.stack(Y)

    def random_policy(self, x):
        a = np.array([0.,0.])
        if np.random.uniform(0.,1.,1) > 0.6:
            if np.random.uniform(0.,1.,1) > 0.5:
                a = np.random.uniform(0.75,1.0,2)
            else:
                a = np.random.uniform(-1.0,-0.75,2)
        else:
            a = np.random.uniform(-1.,1.,2)
            
            
        return np.append(a, np.random.uniform(-1,1,1))

    def pilco_policy(self, x):
        return self.pilco.compute_action(x[None, :])[0, :]

    def callbackGripperStatus(self, msg):
        self.gripper_closed = msg.data == "closed"

    def normz(self, X):
        for i in range(self.state_dim):
            X[:,i] = (X[:,i]-self.state_min[i])/(self.state_max[i]-self.state_min[i])

        return X





if __name__ == '__main__':
    try:
        pilco_node()
    except rospy.ROSInterruptException:
        pass

