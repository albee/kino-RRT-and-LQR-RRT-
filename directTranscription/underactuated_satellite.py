# -*- coding: utf8 -*-
'''
Satellite dynamics and direct transcription trajectory Optimization

Hailee Hettrick
'''

import numpy as np
import math
import csv
import os

from pydrake.all import MathematicalProgram, Solve, IpoptSolver, SolverOptions
from pydrake.math import sin, cos

import matplotlib.pyplot as plt

class underactuatedSatellite():

    def __init__(self):
        self.mass = 4.1  # satellite mass [kg]
        self.len  = 0.25 # side length [m]
        self.MOI  = (1.0 / 6.0) * self.mass * self.len**2  # rotational inertia [kg*m^2]
        self.Fmax = 0.1  # maximum thrust output [N]
        self.pmax = 50.0 # maximum position limit [m]
        self.vmax = 50.0 # maximum velocity limit [m/s]
        self.hmax = 5.0*math.pi #5.0*math.pi  # maximum heading limit [rad]
        self.wmax = math.pi  # maximum angular velocity limit [rad/s]
        self.dep = np.array([1., 1., 1., 1.])
        self.goalState = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def satellite_dynamics(self, state, u):

        '''
        state: x, y, vx, vy, h, w
        '''

        satellite_position = state[0:2]
        derivs = np.zeros_like(state)
        derivs[0:2] = state[2:4]
        satellite_heading = state[4]
        satellite_angvel = state[5]

        m = self.mass; l = self.len; I = self.MOI; dep = self.dep;

        # Underactuated system via dep array
        derivs[2] = (u[0]*dep[0]+u[1]*dep[1]-u[2]*dep[2]-u[3]*dep[3])*sin(satellite_heading) / m # solves for dvx 
        derivs[3] = (-u[0]*dep[0]-u[1]*dep[1]+u[2]*dep[2]+u[3]*dep[3])*cos(satellite_heading) / m # solves for dvy
        derivs[4] = satellite_angvel # solves for dh
        derivs[5] = (-u[0]*dep[0]+u[1]*dep[1]-u[2]*dep[2]+u[3]*dep[3]) * l / (2.0*I) # solves for dw

        return derivs

    def passive_satellite_dynamics(self, state):

        u = np.zeros(4)
        return self.satellite_dynamics(state,u)

    def simulate_states_over_time(self, state_initial, time_array, input_trajectory):
        '''
        Given an initial state, simulates the state of the system.

        :param state_initial: numpy array of length 6, see satellite_dynamics for documentation
        :param time_array: numpy array of length N+1 (0, ..., N) whose elements are samples in time, i.e.:
            [ t_0,
              ...
              t_N ] 
            Note the times do not have to be evenly spaced
        :param input_trajectory: numpy 2d array of N rows (0, ..., N-1), and 4 columns, corresponding to
            the control inputs at each time, except the last time, i.e.:
            [ [u_0, u_1, u_2, u_3],
              ...
              [u_{N-1}, u_{N-1}, u_{N-1}, u_{N-1}] ]

        :return: numpy 2d array where the rows are samples in time corresponding
            to the time_array, and each row is the state at that time, i.e.:
            [ [x_0, y_0, \dot{x}_0, \dot{y}_0, h_0, w_0],
              ...
              [x_N, y_N, \dot{x}_N, \dot{y}_N], h_N, w_N]
        '''
        states_over_time = np.asarray([state_initial])
        for i in range(1,len(time_array)):
            time_step = time_array[i] - time_array[i-1]
            state_next = states_over_time[-1,:] + time_step*self.satellite_dynamics(states_over_time[-1,:], input_trajectory[i-1,:])
            states_over_time = np.vstack((states_over_time, state_next))
        return states_over_time

    def simulate_states_over_time_passive(self, state_initial, time_array):
        '''
        Given an initial state, simulates the state of the system passively

        '''
        input_trajectory = np.zeros((len(time_array)-1,4))
        return self.simulate_states_over_time(state_initial, time_array, input_trajectory)

    def plot_trajectory(self, trajectory):
        '''
        Given a trajectory, plots this trajectory over time.

        :param: trajectory: the output of simulate_states_over_time, or equivalent
            Note: see simulate_states_over_time for documentation of the shape of the output
        '''
        input_trajectory = np.zeros((trajectory.shape[0],4))
        self.plot_trajectory_with_boosters(trajectory, input_trajectory)

    def plot_trajectory_with_boosters(self, trajectory, input_trajectory):
        '''
        Given a trajectory and an input_trajectory, plots this trajectory and control inputs over time.

        :param: trajectory: the output of simulate_states_over_time, or equivalent
            Note: see simulate_states_over_time for documentation of the shape of the output
        :param: input_trajectory: the input to simulate_states_over_time, or equivalent
            Note: see simulate_states_over_time for documentation of the shape of the input_trajectory
        '''
        satellite_position_x = trajectory[:,0]
        satellite_position_y = trajectory[:,1]
        fig, axes = plt.subplots(nrows=1,ncols=1)
        axes.plot(satellite_position_x, satellite_position_y)
        axes.axis('equal')
        plt.plot(satellite_position_x[0],satellite_position_y[0],"or")
        plt.plot(satellite_position_x[-1],satellite_position_y[-1],"ok")
        plt.grid(True)

        ## if we have an input trajectory, plot it
        if len(input_trajectory.nonzero()[0]):
            # the quiver plot works best with not too many arrows
            max_desired_arrows = 40
            num_time_steps = input_trajectory.shape[0]

            if num_time_steps < max_desired_arrows:
                downsample_rate = 1 
            else: 
                downsample_rate = num_time_steps / max_desired_arrows

            u_traj = input_trajectory[::downsample_rate,:]

            satellite_position_x = satellite_position_x[:-1] # don't need the last state, no control input for it
            satellite_position_y = satellite_position_y[:-1]
            satellite_booster_x = u_traj[:,0] + u_traj[:,1] 
            satellite_booster_y = u_traj[:,2] + u_traj[:,3]
            Q = plt.quiver(satellite_position_x[::downsample_rate], satellite_position_y[::downsample_rate], \
                satellite_booster_x, satellite_booster_y, units='width', color="red")


        plt.show()

    #### direct transcription ####
    def compute_trajectory(self, state_initial, minimum_time, maximum_time):
        '''
        :param: state_initial: :param state_initial: numpy array of length 6, see satellite_dynamics for documentation
        :param: minimum_time: float, minimum time allowed for trajectory
        :param: maximum_time: float, maximum time allowed for trajectory

        :return: three return args separated by commas:

            trajectory, input_trajectory, time_array

            trajectory: a 2d array with N rows, and 6 columns. See simulate_states_over_time for more documentation.
            input_trajectory: a 2d array with N-1 row, and 4 columns. See simulate_states_over_time for more documentation.
            time_array: an array with N rows. 

        '''
        mp = MathematicalProgram()

        # use direct transcription
        N = 50
        #N = 200

        # Unpack state bounds
        pmax = self.pmax
        vmax = self.vmax
        hmax = self.hmax
        wmax = self.wmax

        # Calculate number of states and control input
        nq = len(state_initial)
        nu = 4

        #### BEGIN: Decision Variables ####
        # Inputs
        k = 0
        u = mp.NewContinuousVariables(nu,"u_%d" % k)
        u_over_time = u

        for k in range(1,N-1):
            u = mp.NewContinuousVariables(nu, "u_%d" % k)
            u_over_time = np.vstack((u_over_time, u))

        # States
        k = 0
        states = mp.NewContinuousVariables(nq,"states_over_time_%d" % k)
        states_over_time = states

        for k in range(1,N):
            states = mp.NewContinuousVariables(nq, "states_over_time_%d" % k)
            states_over_time = np.vstack((states_over_time, states))
        
        # Final Time
        tf = mp.NewContinuousVariables(1,"tf")
        dt = tf / (N-1)
        #### END: Decision Variables ####

        #### BEGIN: Input constraints ####
        for i in range(N-1):  
            for j in range(nu):
                mp.AddConstraint(u_over_time[i,j] >= 0.0)
                mp.AddConstraint(u_over_time[i,j] <= 0.10)
        #### END: Input constraints ####        

        #### BEGIN: Time constraints ####
        # Final time must be between minimum_time and maximum_time
        mp.AddConstraint(tf[0] <= maximum_time)
        mp.AddConstraint(tf[0] >= minimum_time)
        #### END: Time constraints ####

        #### BEGIN: State constraints ####
        # initial state constraint
        for j in range(nq):
            mp.AddLinearConstraint(states_over_time[0,j] >= state_initial[j])
            mp.AddLinearConstraint(states_over_time[0,j] <= state_initial[j])

        # state constraints for all time
        for i in range(N):
            mp.AddLinearConstraint(states_over_time[i,0] <= pmax)
            mp.AddLinearConstraint(states_over_time[i,0] >= -pmax)
            mp.AddLinearConstraint(states_over_time[i,1] <= pmax)
            mp.AddLinearConstraint(states_over_time[i,1] >= -pmax)
            mp.AddLinearConstraint(states_over_time[i,2] <= vmax)
            mp.AddLinearConstraint(states_over_time[i,2] >= -vmax)
            mp.AddLinearConstraint(states_over_time[i,3] <= vmax)
            mp.AddLinearConstraint(states_over_time[i,3] >= -vmax)
            mp.AddLinearConstraint(states_over_time[i,4] <= hmax)
            mp.AddLinearConstraint(states_over_time[i,4] >= -hmax)
            mp.AddLinearConstraint(states_over_time[i,5] <= wmax)
            mp.AddLinearConstraint(states_over_time[i,5] >= -wmax)

        # dynamic constraint
        for i in range(N-1):
            dx = self.satellite_dynamics(states_over_time[i,:],u_over_time[i,:])
            for j in range(nq):
                mp.AddConstraint(states_over_time[i+1,j]<=states_over_time[i,j]+dx[j]*dt[0])
                mp.AddConstraint(states_over_time[i+1,j]>=states_over_time[i,j]+dx[j]*dt[0])

        # Final state constraint
        final_state_error = states_over_time[-1,:] - self.goalState
        mp.AddConstraint((final_state_error).dot(final_state_error) <= 0.001**2)
        #### END: State constraints ####

        #### BEGIN: Costs ####
        # Quadratic cost on fuel (aka control)
        for j in range(nu):
        	mp.AddQuadraticCost(u_over_time[:,j].dot(u_over_time[:,j]))

        #### END: Costs ####

        #### Solve the Optimization! ####
        result = Solve(mp)
        #print result.is_success()

        #### Name outputs appropriately ####
        # Time - knot points
        optimal_tf = result.GetSolution(tf)
        time_step = optimal_tf / (N-1)
        time_array = np.arange(0.0,optimal_tf+time_step,time_step)

        # Allocate to input trajectory
        input_trajectory = result.GetSolution(u_over_time)

        # Allocate to trajectory output
        trajectory = result.GetSolution(states_over_time)

        # save to csv
        if os.path.exists("traj.csv"):
            os.remove("traj.csv")

        if os.path.exists("input_traj.csv"):
            os.remove("input_traj.csv")

        with open('traj.csv', 'a') as csvFile:
        	writer = csv.writer(csvFile)
        	for i in range(N):
        		row = [time_array[i], trajectory[i,0], trajectory[i,1], trajectory[i,2], trajectory[i,3], trajectory[i,4], trajectory[i,5]]
        		writer.writerow(row)

        with open('input_traj.csv', 'a') as csvFile:
        	writer = csv.writer(csvFile)
        	for i in range(N-1):
        		row = [input_trajectory[i,0], input_trajectory[i,1], input_trajectory[i,2], input_trajectory[i,3]]
        		writer.writerow(row)

        csvFile.close()
 
        return trajectory, input_trajectory, time_array
