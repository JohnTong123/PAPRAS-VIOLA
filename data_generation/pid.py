import math
import numpy as np
class basicPID:

    def __init__(self, kp=6 ,ki=0.5, kd=0.1):
        self.ki = ki
        self.kp = kp
        self.kd = kd
        self.accumulated_error = np.array([0,0,0,0],dtype='float64')
        self.last_error= np.array([0,0,0,0],dtype='float64')
        self.tau = 0.02
        # print(self.tau)
        # print('herro')
    def update(self, value, target):
        error = target - value
        # print(self.tau)
        self.accumulated_error += (self.last_error + error) * 0.5 * self.tau  #(self.last_error * self.tau) + self.tau * (error - self.last_error) * 0.5
        output = self.kp * error + self.accumulated_error * self.ki + (error-self.last_error) * self.kd/self.tau
        # output = 
        self.last_error = error
        # print(output)
        # print(output)
        # print('aaaa')
        print(value,"val")
        print(output, 'out')
        return np.clip(output, -5,5)

    def reset(self):
        self.accumulated_error = np.array([0,0,0,0],dtype='float64')
        self.last_error = np.array([0,0,0,0],dtype='float64')

        #[ 0.05221125 -0.19450871  1.15842029]
# 0 0 1.2
# 0.1 0 1.15
# 0.1 -0.08 1.1
# 0.1 -0.15 1.05
# 0.1 -0.19 1.03
# 0.1 -0.195 0.96
# 0.1 -0.195 0.91
# 0.1 -0.195 0.915 1


# 0 0 1.2
# 0 -0.08 1.2
# 0 -0.15 1.1
# 0 -0.18 1.1
# 0 -0.2 1.05
# 0 -0.2 1.02 1