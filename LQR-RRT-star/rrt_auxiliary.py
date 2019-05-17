'''
Support functions for RRT.
'''

import math
from tree_auxiliary import *
import Dynamics3DoF as dynamics
import numpy as np
import scipy.linalg as linalg
import random


# Method to calculate nearest node to randomly generated state in existing tree
def nearest(V, rand_state):

    # Unpack randomly generated state
    x_rand  = rand_state[0]
    y_rand  = rand_state[1]
    vx_rand = rand_state[2]
    vy_rand = rand_state[3]
    h_rand  = rand_state[4]
    w_rand  = rand_state[5]

    # Initialize variable to record shortest distance
    d_min = 0

    # Initialize variable to record closest node
    nearest_node = []

    # Calculate LQR cost matrix
    p = calc_cost_matrix(rand_state)

    # Iterate through all nodes and determine the closest
    for node in V:

        # Unpack node state
        state = node.state
        x_node  = state[0]
        y_node  = state[1]
        vx_node = state[2]
        vy_node = state[3]
        h_node  = state[4]
        w_node  = state[5]

        # Calculate distance to randomly generated node
        # Below will change to LQR cost eventually
        #d = math.sqrt((x_rand-x_node)**2 + (y_rand-y_node)**2 + (vx_rand-vx_node)**2 + (vy_rand-vy_node)**2 + (h_rand-h_node)**2  +(w_rand-w_node)**2)
        d = lqr_distance(node.state, rand_state,p)

        # Store if shorter than the shortest distance
        if d < d_min or d_min == 0:
            d_min = d
            nearest_node = node

    # Return nearest node
    return nearest_node


# Method to steer robot in direction of the randomly generated state
def steer(nearest_node, rand_state, d_max, ctrl_bounds, dt):

    # Define number of samples
    N = 500

    # Calculate LQR cost matrix
    p = calc_cost_matrix(rand_state)

    # Unpack nearest node
    state = nearest_node.state
    nq = len(state)

    # Calculate new node position - Evaluate forward dynamics
    # iterate over various inputs; intermediate values or randomize between max and min
    my_sat = dynamics.Dynamics3DoF()
    # FIX: don't hardcode these
    umin = 0.
    umax = 1.
    dt = .10
    # FIX: save input trajectory
    # Control bounds
    umin = ctrl_bounds[0] 
    umax = ctrl_bounds[1] 
    # Create zeros
    dotx = np.zeros((nq, N))
    delta = np.zeros((nq, N))
    state_new = np.zeros((nq, N))
    dist = np.zeros(N)
    temp_sq = np.zeros(N)
    u = np.zeros((4,N))
    u[:,0:16] = control_permutations(umin, umax)
    u[:,16:32] = control_permutations(umin, umax/2.0)
    u[:,32:48] = control_permutations(umin, umax/3.0)
    u[:,48:64] = control_permutations(umin, umax/4.0)
    u[:,64:80] = control_permutations(umin, umax/5.0)
    u[:,80:96] = control_permutations(umin, umax/10.0)

    for j in range(N):
        if j >= 96:
            for i in range(2):

                u1 = random.uniform(umin,umax)
                u2 = random.uniform(umin,umax)

                if random.uniform(0, 1) < 0.5:
                    u[0,j] = u1
                    u[3,j] = 0
                else:
                    u[0,j] = 0
                    u[3,j] = u1

                if random.uniform(0, 1) < 0.5:
                    u[1,j] = u2
                    u[2,j] = 0
                else:
                    u[1,j] = 0
                    u[2,j] = u2

        dotx[:,j] = my_sat.satellite_dynamics(state, u[:,j])
        state_new[:,j] = state[:] + dt*dotx[:,j]
        for k in range(nq):
            delta[k,j] = state_new[k,j] - rand_state[k]
            temp_sq[k] = delta[k,j]**2
        dist[j] = lqr_distance(state_new[:,j], rand_state, p)

    # find which iteration came closest to rand
    dist_min = np.where(dist == np.amin(dist))
    idx = dist_min[0]
    idx = idx[0]
    state = state_new[:,idx]
    state = np.squeeze(state)
    control = u[:,idx]

    # Calculate new node cost
    cost = nearest_node.cost + dist[idx]

    # Create new node
    new_node = SearchNode(state, nearest_node, cost, control)

    # Return new node
    return new_node


def lqr_distance(state1, state2, p=None):

    # Cast states to numpy arrays
    state1 = np.array(state1)
    state2 = np.array(state2)

    if p is None:
        p = calc_cost_matrix(state2)

    # Calculate state error
    x_error = state1 - state2

    # Calculate LQR cost
    lqr_cost = np.transpose(x_error).dot(p.dot(x_error))

    # Return LQR cost as distance
    return lqr_cost


def calc_cost_matrix(state):

    # Unpack target state
    x = state[0]
    y = state[1]
    u = state[2]
    v = state[3]
    h = state[4]
    w = state[5]

    # Hardcode mass for now (need to fix)
    m = 4.1
    l = 0.25
    I = (1./6.)*m*l**2

    # Define slack
    eps = 0.

    # Define dynamics metrics
    a = np.array(((0, 0, 1, 0, 0, 0),
                  (0, 0, 0, 1, 0, 0),
                  (0, 0, 0, 0, -math.sin(h)/m, 0),
                  (0, 0, 0, 0, math.cos(h)/m, 0),
                  (0, 0, 0, 0, 0, 1),
                  (0, 0, 0, 0, 0, 0)))

    b = np.array(((0, 0, 0, 0),
                  (0, 0, 0, 0),
                  (math.cos(h) / m + eps, math.cos(h) / m - eps, -math.cos(h) / m + eps, -math.cos(h) / m - eps),
                  (-math.sin(h) / m + eps, -math.sin(h) / m - eps, math.sin(h) / m + eps, math.sin(h) / m - eps),
                  (0, 0, 0, 0),
                  (-I/2/l, I/2/l, -I/2/l, I/2/l)))

    # Define LQR weighting matrices
    q = np.diagflat([10, 10, 1, 1, 10, 1])
    r = 100 * np.identity(4)

    # Solve algebraic Riccati equation
    p = linalg.solve_continuous_are(a, b, q, r)

    # Return LQR cost matrix
    return p


def control_permutations(umin, u_value):
    control_inputs = np.zeros((4,16))
    control_inputs[:,0] = np.asarray([u_value, u_value, umin, umin])
    control_inputs[:,1] = np.asarray([umin, umin, u_value, u_value])
    control_inputs[:,2] = np.asarray([u_value, umin, u_value, umin])
    control_inputs[:,3] = np.asarray([umin, u_value, umin, u_value])
    control_inputs[:,4] = np.asarray([umin, umin, umin, umin])
    control_inputs[:,5] = np.asarray([u_value, u_value, u_value, u_value])
    control_inputs[:,6] = np.asarray([u_value, umin, umin, umin])
    control_inputs[:,7] = np.asarray([umin, u_value, umin, umin])
    control_inputs[:,8] = np.asarray([umin, umin, u_value, umin])
    control_inputs[:,9] = np.asarray([umin, umin, umin, u_value])
    control_inputs[:,10] = np.asarray([u_value, umin, umin, u_value])
    control_inputs[:,11] = np.asarray([umin, u_value, u_value, umin])
    control_inputs[:,12] = np.asarray([u_value, u_value, umin, u_value])
    control_inputs[:,13] = np.asarray([u_value, umin, u_value, u_value])
    control_inputs[:,14] = np.asarray([umin, u_value, u_value, u_value])
    control_inputs[:,15] = np.asarray([u_value, u_value, u_value, umin])

    return control_inputs


def in_bounds(state, bounds):

    # Unpack state
    x = state[0]
    y = state[1]
    u = state[2]
    v = state[3]
    h = state[4]
    w = state[5]

    # Unpack bounds
    x_min = bounds[0]
    x_max = bounds[1]
    y_min = bounds[2]
    y_max = bounds[3]
    vx_min = bounds[4]
    vx_max = bounds[5]
    vy_min = bounds[6]
    vy_max = bounds[7]
    h_min = bounds[8]
    h_max = bounds[9]
    w_min = bounds[10]
    w_max = bounds[11]

    # Check bounds
    if x < x_min or x > x_max:
        return False

    if y < y_min or y > y_max:
        return False

    if u < vx_min or u > vx_max:
        return False

    if v < vy_min or v > vy_max:
        return False

    if h < h_min or h > h_max:
        return False

    if w < w_min or w > w_max:
        return False

    return True
