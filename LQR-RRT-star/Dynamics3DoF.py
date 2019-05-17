'''
Dynamics and supporting functions for an underactuated satellite.
Includes animation routines.

Hailee Hettrick and Keenan Albee
'''

from numpy import sin, cos
import numpy as np
import math
import time
import matplotlib.animation as anim
import matplotlib.patches as patches
import matplotlib.pyplot as plt

class Dynamics3DoF():
    def __init__(self):
        self.mass = 4.1  # satellite mass [kg]
        self.len  = 0.25 # side length [m]
        self.MOI  = (1.0 / 6.0) * self.mass * self.len**2  # rotational inertia [kg*m^2]
        self.Fmax = 1.  # maximum thrust output [N]
        self.pmax = .5 # maximum position limit [m]
        self.vmax = .5 # maximum velocity limit [m/s]
        self.hmax = math.pi  # maximum heading limit [rad]
        self.wmax = 2.  # maximum angular velocity limit [rad/s]
    
    '''
    state: [x, y, vx, vy, theta, w]:
        theta is a positive rotation of the base relative to INERTIAL frame
        x, y, vx, by, and w are scalars from their corresponding vectors expressed in the INERTIAL frame
    u: [4x thrusters]: inputs are in the BODY frame
    '''
    def satellite_dynamics(self, state, u):
        satellite_position = state[0:2]
        theta = state[4]
        satellite_angvel = state[5]

        m = self.mass; l = self.len; I = self.MOI;

        # mixer: convert thruster inputs to force/torque in the INERTIAL frame
        # 4 thrusters: 4xu -> 3xF/T, fully actuated
        M = np.asarray( \
            [[sin(theta), sin(theta), -sin(theta), -sin(theta)],  # F_x
             [-cos(theta), -cos(theta), cos(theta), cos(theta)],  # F_y
             [-l/2.0, l/2.0, -l/2.0, l/2.0]]  # T_z
        )

        F = u[2]+u[3]-u[0]-u[1]

        # state matrix
        A = np.asarray( \
            [[0, 0, 1, 0, 0, 0],
             [0, 0, 0, 1, 0, 0],
             [0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 1],
             [0, 0, 0, 0, 0, 0]]
        )

        # direct force/torque commands
        B = np.array( \
                [[0, 0, 0],
                 [0, 0, 0],
                 [1/m, 0, 0],
                 [0, 1/m, 0],
                 [0, 0, 0],
                 [0, 0, 1/I]]
        )

        dot_x = np.dot(A, state) + np.dot(np.dot(B, M), u)

        return dot_x

    def passive_satellite_dynamics(self, state):
        u = np.zeros(4)
        return self.satellite_dynamics(state, u)

    '''
    Given an initial state, simulates the state of the system. This uses simple Euler integration.

    The first time of the time_vec is the time of the state_initial.

    :param state_initial: numpy array of length 6, see satellite_dynamics for documentation
    :param time_vec: numpy array of length N+1 (0, ..., N) whose elements are samples in time, i.e.:
        [ t_0,
          ...
          t_N ] 
        Note the times do not have to be evenly spaced
    :param input_array: numpy 2d array of N rows (0, ..., N-1), and dim(u) columns, corresponding to
        the control inputs at each time, except the last time, i.e.:
        [ [u_0, u_1, u_2, u_3],
          ...
          [u_{N-1}, u_{N-1}, u_{N-1}, u_{N-1}] ]

    :return: numpy 2d array where the rows are samples in time corresponding
        to the time_vec, and each row is the state at that time, i.e.:
        [ [x_0, y_0, dot{x}_0, dot{y}_0, h_0, w_0],
          ...
          [x_N, y_N, dot{x}_N, dot{y}_N], h_N, w_N]

    Adapted from 6.832.
    '''
    def simulate_trajectory(self, state_initial, time_vec, input_array):
        trajectory = np.asarray([state_initial])

        for i in range(1,len(time_vec)):
            time_step = time_vec[i] - time_vec[i-1]
            # euler integrate
            state_next = trajectory[-1,:] + time_step*self.satellite_dynamics(trajectory[-1,:], input_array[i-1,:])
            trajectory = np.vstack((trajectory, state_next))
        return trajectory

    '''
    Given an initial state, simulates the state of the system passively.
    '''
    def simulate_trajectory_passive(self, state_initial, time_vec):
        input_array = np.zeros((len(time_vec)-1,4))
        return self.simulate_trajectory(state_initial, time_vec, input_array)

    '''
    Given a trajectory, plots this trajectory over time.

    :param: trajectory: a [nx6] np array
    '''
    def plot_trajectory_passive(self, trajectory):
        input_array = np.zeros((trajectory.shape[0],4))
        self.plot_trajectory(trajectory, input_array)

    def plot_trajectory(self, trajectory, input_array):
        plot_trajectory(trajectory, input_array, self.Fmax)



'''
Given a trajectory and an input_array, plots this trajectory and control inputs over time.

:param: trajectory: the output of simulate_trajectory, or equivalent
Note: see simulate_trajectory for documentation of the shape of the output
:param: input_array: the input to simulate_trajectory, or equivalent
Note: see simulate_trajectory for documentation of the shape of the input_array
'''
def plot_trajectory(trajectory, input_array, input_max):
    max_desired_arrows = 40  # the quiver plot works best with not too many arrows
    pos_x = trajectory[:,0]
    pos_y = trajectory[:,1]
    theta_z = trajectory[:,4]

    # heading setup
    dir_u = np.array([])
    dir_v = np.array([])
    for each in theta_z:
        R_I_B = rot_2D(each)
        dir_u = np.append(dir_u, np.dot(R_I_B, np.array([1, 0]))[0] )
        dir_v = np.append(dir_v, np.dot(R_I_B, np.array([1,0]))[1] )

    # actual plotting
    fig, axes = plt.subplots(nrows=1,ncols=1)
    axes.axis('equal')
    plt.ylim(-1, 1)
    plt.xlim(-1, 1)

    # plot full traj once
    axes.plot(pos_x, pos_y)
    # axes.quiver(pos_x, pos_y, dir_u, dir_v)
    plt.plot(pos_x[0],pos_y[0],"or")  # start state
    plt.plot(pos_x[-1],pos_y[-1],"ok")  # final state

    animate_rect(fig, axes, pos_x, pos_y, theta_z, input_array, input_max, collage=False, record=False)
    plt.pause(100)

# 2D counterclockwise rotation matrix   
def rot_2D(theta):
    R = np.array( \
        [[cos(theta), -sin(theta)],
         [sin(theta), cos(theta)]]
        )
    return R

# Note argument order
def pose_transform_2D(r_B, r_IB_I, theta_I_B):
    R = rot_2D(theta_I_B)
    r_I = np.matmul(R, r_B) + r_IB_I
    return r_I

# Update a plot object to (erase old rectangle and) plot new rectangle
# CENTERED on x and y, with heading about center, h
def plot_rectangle(cur_axes, x, y, h, width, height, plot_now):
    # convert to world frame
    xy_lower_left_B = np.array([-width/2, -height/2])
    xy_lower_left_I = pose_transform_2D(xy_lower_left_B, np.array([x, y]), h)
    x_I = xy_lower_left_I[0]  # lower left corner
    y_I = xy_lower_left_I[1]  # lower left corner

    patch = patches.Rectangle((x_I, y_I), width, height, np.rad2deg(h), ec='black',fc='None')

    if plot_now == True:
        cur_axes.add_patch(patch)

    return patch

'''
Animate a series of poses[ ] on cur_axes

Call as:
animate_rect(axes, sc_poses, input_array, collage=False)
OR
animate_rect(axes, sc_poses, collage=False)
'''
def animate_rect(cur_fig, cur_axes, pos_x, pos_y, theta_z, *args, **kwargs):
    width = 0.1
    height = width + 0.1

    # in the more general case, should embed this info in Dynamics
    pos_thruster1_B = np.array([width/2, height/2])
    pos_thruster2_B = np.array([-width/2, height/2])
    pos_thruster3_B = np.array([-width/2, -height/2])
    pos_thruster4_B = np.array([width/2, -height/2])

    dir_thruster1_B = np.array([0, 1])
    dir_thruster2_B = np.array([0, 1])
    dir_thruster3_B = np.array([0, -1])
    dir_thruster4_B = np.array([0, -1])

    if len(args) > 0:
        input_array = args[0]
        u_max = args[1]
        u_max = 50.0*u_max

    # moviewriter setup
    FFMpegWriter = anim.writers['ffmpeg']
    metadata = dict(title='Movie Test', artist='Matplotlib',
        comment='Movie support!')
    writer = FFMpegWriter(fps=8, metadata=metadata)

    collage = False
    record = False
    for key, value in kwargs.items():
        if key=="collage":
            collage = value
        if key=="record":
            record = value

    # get a list of plottable sc_poses
    sc_poses = []
    for i in range(pos_x.shape[0]):
        new_rect = plot_rectangle(cur_axes, pos_x[i], pos_y[i], theta_z[i], width, height, plot_now=False)
        sc_poses.append(new_rect)

    # loop and animate
    with writer.saving(cur_fig, "writer_test.mp4", 400):
        for i in range(len(sc_poses)):
            if (collage == False) and (i > 0):  # remove last
                sc_poses[i-1].remove()
            cur_axes.add_patch(sc_poses[i])

            if len(args) > 0:  # plot inputs
                u_vec = input_array[i,:]
                if (collage == False) and (i > 0):  # remove last
                    qu1.remove()
                    qu2.remove()
                    qu3.remove()
                    qu4.remove()
                r_IB_I = np.array([pos_x[i], pos_y[i]])
                pos_thruster1_I = pose_transform_2D(pos_thruster1_B, r_IB_I, theta_z[i])
                pos_thruster2_I = pose_transform_2D(pos_thruster2_B, r_IB_I, theta_z[i])
                pos_thruster3_I = pose_transform_2D(pos_thruster3_B, r_IB_I, theta_z[i])
                pos_thruster4_I = pose_transform_2D(pos_thruster4_B, r_IB_I, theta_z[i])

                dir_thruster1_I = pose_transform_2D(dir_thruster1_B, np.zeros(2), theta_z[i])*u_vec[0]/u_max
                dir_thruster2_I = pose_transform_2D(dir_thruster2_B, np.zeros(2), theta_z[i])*u_vec[1]/u_max
                dir_thruster3_I = pose_transform_2D(dir_thruster3_B, np.zeros(2), theta_z[i])*u_vec[2]/u_max
                dir_thruster4_I = pose_transform_2D(dir_thruster4_B, np.zeros(2), theta_z[i])*u_vec[3]/u_max

                qu1 = cur_axes.quiver(pos_thruster1_I[0], pos_thruster1_I[1], dir_thruster1_I[0], dir_thruster1_I[1], scale_units="height", scale=10.0)
                qu2 = cur_axes.quiver(pos_thruster2_I[0], pos_thruster2_I[1], dir_thruster2_I[0], dir_thruster2_I[1], scale_units="height", scale=10.0)
                qu3 = cur_axes.quiver(pos_thruster3_I[0], pos_thruster3_I[1], dir_thruster3_I[0], dir_thruster3_I[1], scale_units="height", scale=10.0)
                qu4 = cur_axes.quiver(pos_thruster4_I[0], pos_thruster4_I[1], dir_thruster4_I[0], dir_thruster4_I[1], scale_units="height", scale=10.0)
           
            # record or pause and show
            if record:
                writer.grab_frame()
            else:
                plt.pause(0.05)
    plt.show()