'''
Support functions for RRT.
'''

import math
from tree_auxiliary import *
import Dynamics3DoF as dynamics
import numpy as np
import scipy.linalg as linalg
import random
from plot_auxiliary import *


# Method to calculate nearest node to randomly generated state in existing tree, using
# linearized LQR cost-to-go about rand_state
def LQR_nearest(V, rand_state):
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
    P = calc_cost_matrix(rand_state)

    # Iterate through all nodes and determine the closest
    for node in V:
        # Unpack node state
        state = node.state

        # Calculate distance to randomly generated node
        d = lqr_distance(node.state, rand_state, P)

        # Store if shorter than the shortest distance
        if d < d_min or d_min == 0:
            d_min = d
            nearest_node = node

    # Return nearest node
    return nearest_node

# Method to calculate nearest node to randomly generated state in existing tree,
# using L2-norm
def L2_nearest(V, to_state):

    # Unpack randomly generated state
    x_rand  = to_state[0]
    y_rand  = to_state[1]
    vx_rand = to_state[2]
    vy_rand = to_state[3]
    h_rand  = to_state[4]
    w_rand  = to_state[5]

    # Initialize variable to record shortest distance
    d_min = 0

    # Initialize variable to record closest node
    nearest_node = []

    # Calculate LQR cost matrix
    P = calc_cost_matrix(to_state)

    # Iterate through all nodes and determine the closest
    for node in V:
        # Unpack node state
        state = node.state

        # Calculate distance to randomly generated node
        d = L2_distance(state, to_state)

        # Store if shorter than the shortest distance
        if d < d_min or d_min == 0:
            d_min = d
            nearest_node = node

    # Return nearest node
    return nearest_node


# Method to steer robot in direction of the randomly generated state
# Uses motion primitive steering.
def guided_steer(nearest_node, rand_state, d_max, ctrl_bounds, dt, t_step, bounds):
    # Calculate LQR cost matrix
    P = calc_cost_matrix(rand_state)

    nearest_state = nearest_node.state
    my_sat = dynamics.Dynamics3DoF()
    control_options = control_permutations(my_sat.Fmax/8, my_sat.Fmax)

    min_dist = float('inf')
    return_state = nearest_state

    for i in range(control_options.shape[1]):
        u_optimal = control_options[:,i]

        # Euler integration loop
        new_state = nearest_state
        t_k = 0.0
        while t_k < t_step:
            for i in range(len(u_optimal)):
                if u_optimal[i] < 0:
                    u_optimal[i] = 0
                if u_optimal[i] > my_sat.Fmax:
                    u_optimal[i] = my_sat.Fmax

            # Calculate new node position - Evaluate forward dynamics
            dotx = my_sat.satellite_dynamics(new_state, u_optimal)
            new_state_temp = new_state + dt*dotx

            if not in_bounds(new_state_temp, bounds) and t_k > dt:
                break
            new_state = new_state_temp

            # wrap angle
            theta = new_state[4]
            theta = theta%(2*math.pi)
            if theta > math.pi:
                theta -= 2*math.pi
            new_state[4] = theta
            t_k += dt

        option_dist = lqr_distance(new_state, rand_state, P)
        if option_dist < min_dist:
            min_dist = option_dist
            return_state = new_state

    control = 'TODO'  # save control

    # Calculate new node cost
    cost = nearest_node.cost + min_dist #dist[idx]

    # Create new node
    new_node = SearchNode(return_state, nearest_node, cost, control)

    # Return new node
    return new_node

# Method to steer robot in direction of the randomly generated state
# Uses the linearized infinite horizon LQR policy.
def LQR_steer(nearest_node, rand_state, d_max, ctrl_bounds, dt, t_step, bounds):
    # Calculate LQR cost matrix
    P = calc_cost_matrix(rand_state)

    # Unpack nearest node
    nearest_state = nearest_node.state
    my_sat = dynamics.Dynamics3DoF()

    ### LQR policy evaluation
    K = calc_K_matrix(rand_state, P)  # believe this should be around rand_state, otherwise not sure how to interpret

    # Euler integration loop
    new_state = nearest_state
    t_k = 0.0
    while t_k < t_step:
        # Use LQR policy about x_rand
        u_optimal = -K.dot(new_state - rand_state)
        max_mag = np.amax(np.absolute(u_optimal))
        if max_mag > my_sat.Fmax:
            u_optimal = u_optimal/max_mag
            # u_optimal = np.clip(u_optimal, -my_sat.Fmax, my_sat.Fmax)

        # Calculate new node position - Evaluate forward dynamics
        dotx = my_sat.satellite_dynamics(new_state, u_optimal)
        new_state = new_state + dt*dotx
        # wrap angle
        theta = new_state[4]
        theta = theta%(2*math.pi)
        if theta > math.pi:
            theta -= 2*math.pi
        new_state[4] = theta
        t_k += dt

    # TODO: this is not recording the correct control
    # TODO: make sure control is clipped down properly
    control = u_optimal
    dist = lqr_distance(new_state, rand_state, P)

    # Calculate new node cost
    cost = nearest_node.cost + dist #dist[idx]

    # Create new node
    new_node = SearchNode(new_state, nearest_node, cost, control)

    # Return new node
    return new_node

def random_steer(nearest_node, rand_state, d_max, ctrl_bounds, dt, t_step, bounds):
    # Unpack nearest node
    nearest_state = nearest_node.state
    my_sat = dynamics.Dynamics3DoF()
    control_options = control_permutations(my_sat.Fmax/8, my_sat.Fmax)

    min_dist = float('inf')
    return_state = nearest_state

    for i in range(control_options.shape[1]):
        u_optimal = control_options[:,i]
        # Euler integration loop
        new_state = nearest_state
        t_k = 0.0
        # print(u_optimal)
        # plt.pause(1)
        while t_k < t_step:

            for i in range(len(u_optimal)):
                if u_optimal[i] < 0:
                    u_optimal[i] = 0
                if u_optimal[i] > my_sat.Fmax:
                    u_optimal[i] = my_sat.Fmax

            # break

            # Calculate new node position - Evaluate forward dynamics
            dotx = my_sat.satellite_dynamics(new_state, u_optimal)
            new_state_temp = new_state + dt*dotx

            if not in_bounds(new_state_temp, bounds) and t_k > dt:
                break
            new_state = new_state_temp

            t_k += dt

        option_dist = L2_distance(new_state, rand_state)
        if option_dist < min_dist:
            min_dist = option_dist
            return_state = new_state

    control = u_optimal

    # Calculate new node cost
    cost = nearest_node.cost + min_dist

    # Create new node
    new_node = SearchNode(return_state, nearest_node, cost, control)

    # Return new node
    return new_node

def control_permutations(umin, u_value):
    control_inputs = np.empty((4,0))

    for uval in np.linspace(0, u_value, 3):
        # Doubles, spin
        control_inputs = np.c_[control_inputs, np.asarray([uval, 0, uval, 0])]/5
        control_inputs = np.c_[control_inputs, np.asarray([0, uval, 0, uval])]/5

        # Singles
        control_inputs = np.c_[control_inputs, np.asarray([0, 0, 0, uval])]
        control_inputs = np.c_[control_inputs, np.asarray([0, 0, uval, 0])]
        control_inputs = np.c_[control_inputs, np.asarray([0, uval, 0, 0])]
        control_inputs = np.c_[control_inputs, np.asarray([uval, 0, 0, 0])]

        # Triples
        control_inputs = np.c_[control_inputs, np.asarray([0, uval, uval, uval])]
        control_inputs = np.c_[control_inputs, np.asarray([uval, uval, uval, 0])]
        control_inputs = np.c_[control_inputs, np.asarray([uval, uval, 0, uval])]
        control_inputs = np.c_[control_inputs, np.asarray([uval, 0, uval, uval])]

        # Doubles, translate
        control_inputs = np.c_[control_inputs, np.asarray([uval, uval, 0, 0])]
        control_inputs = np.c_[control_inputs, np.asarray([0, 0, uval, uval])]
    return control_inputs

# Find nodes that are within radius of new_node
def LQR_near(V, E, new_node, radius):
    near_nodes = []
    new_node_state = new_node.state

    for node in V:
        node_state = node.state

        # Calculate distance to randomly generated node
        # dist = np.linalg.norm(new_node_state - node_state)  # L2-norm
        P = calc_cost_matrix(node_state)
        dist = lqr_distance(new_node_state, node_state, P)

         # Store if shorter than radius
        if dist < radius:
            near_nodes.append(node)

    return near_nodes

# Equivalent to choose_parent()
def wire_to_x_new(near_nodes, new_node, nearest_node, E):
    c = 1.0
    min_node = nearest_node
    cost_min = nearest_node.cost + c*line(nearest_node, new_node)

    for near_node in near_nodes:
        cost_to_x_new = near_node.cost + c*line(near_node, new_node)
        if collision_free(near_node, new_node) and (cost_to_x_new < cost_min):
            min_node = near_node
            cost_min = cost_to_x_new

    # update the new_node
    new_node.cost = cost_min
    new_node.parent = min_node
    E.append([min_node, new_node])  # add to edge list

# rewire()
def rewire_from_x_new(near_nodes, new_node, E, fig_num, show_anim):
    c = 1.0
    for near_node in near_nodes:
        cost_from_x_new = new_node.cost + c*line(new_node, near_node)
        if collision_free(new_node, near_node) and (cost_from_x_new < near_node.cost):
            # update E: remove parent, add new_node
            # if [near_node.parent, near_node] in E:
            idx = E.index([near_node.parent, near_node])
            E[idx][0] = new_node
            near_node.cost = cost_from_x_new
            near_node.parent = new_node
            print "rewired performed!"

            # rewire the plot
            if show_anim == True:
                rewire_plot_np_no_goal(idx, fig_num, new_node.state, near_node.state)


# Compute the ball radius of near nodes to check
def get_ball_radius(card_V, state_dimension, volume_obstacle_free, max_dist):
    d = state_dimension
    v = volume_obstacle_free
    if d == 2:
        v_unit_ball = math.pi  # volume of unit ball
    elif d== 6:
        v_unit_ball = math.pi**3/6
    else:
        raise NameError('not yet implemented')

    gamma_star = 2*(1 + 1/d)**(1/d) * (v/v_unit_ball)**1/d
    gamma = gamma_star*1.0 # must be larger than gamma*, provided by Karaman
    return min(gamma * math.log(card_V)/card_V, max_dist)

# LQR cost-to-go between any two points.
def line(from_node, to_node):
    state_from = from_node.state
    state_to = to_node.state

    # cost_to_go = np.linalg.norm(state_from - state_to)
    P = calc_cost_matrix(state_to)
    cost_to_go = lqr_distance(state_from, state_to, P)
    # cost_to_go = L2_distance(state_from, state_to)
    return cost_to_go

# Does collision checking. Not currently implemented.
def collision_free(node_from, node_to):
    return True

# Equivalent to obstacle_free() from [Karaman and Frazzoli, 2011]
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
        # print "x violated"
        return False

    if y < y_min or y > y_max:
        # print "y violated"
        return False

    if u < vx_min or u > vx_max:
        # print "vx violated"
        return False

    if v < vy_min or v > vy_max:
        # print "vy violated"
        return False

    if h < h_min or h > h_max:
        # print "h violated"
        return False

    if w < w_min or w > w_max:
        # print "w violated"
        return False

    return True

# Currently assumes obstacle free
def volume_obstacle_free(bounds, state_dimension):
    d = state_dimension
    if d == 2:
        x_min = bounds[0]
        x_max = bounds[1]
        y_min = bounds[2]
        y_max = bounds[3]
        return (x_max - x_min)*(y_max - y_min)
    elif d == 6:
        x_min = bounds[0]; x_max = bounds[1]; y_min = bounds[2]; y_max = bounds[3]
        vx_min = bounds[4]; vx_max = bounds[5]; vy_min = bounds[6]; vy_max = bounds[7]
        h_min  = bounds[8]; h_max  = bounds[9]; w_min  = bounds[10]; w_max  = bounds[11]
        return (x_max - x_min)*(y_max - y_min)*(vx_max - vx_min)*(vy_max - vy_min)*(h_max - h_min)*(w_max - w_min)
    else:
        raise NameError('Not yet implemented for this state space.')



### LQR and distance metric support

# L2 norm, with tweak for angle
def L2_distance(state_from, state_to):
    h_dist = state_from[4] - state_to[4]
    if h_dist > math.pi: h_dist -= 2*math.pi
    elif h_dist < -math.pi: h_dist += 2*math.pi

    state_from[4] = 0; state_to[4] = 0
    dist = np.linalg.norm(state_from - state_to)

    dist += h_dist*0.0
    return dist

# from, to. LQR relative to state2
def lqr_distance(state_from, state_to, P=None):
    # Cast states to numpy arrays
    state_from = np.array(state_from)
    state_to = np.array(state_to)

    if P is None:
        P = calc_cost_matrix(state_to)

    # Calculate state error
    state_error = state_from - state_to

    # Calculate LQR cost
    lqr_cost = np.transpose(state_error).dot(P.dot(state_error))

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
    A = np.array(((0, 0, 1, 0, 0, 0),
                  (0, 0, 0, 1, 0, 0),
                  (0, 0, 0, 0, math.cos(h)/m, 0),
                  (0, 0, 0, 0, -math.sin(h)/m, 0),
                  (0, 0, 0, 0, 0, 1),
                  (0, 0, 0, 0, 0, 0)))

    B = np.array(((0, 0, 0, 0),
                  (0, 0, 0, 0),
                  (math.sin(h) / m, math.sin(h) / m, -math.sin(h) / m, -math.sin(h) / m),
                  (-math.cos(h) / m, -math.cos(h) / m, math.cos(h) / m, math.cos(h) / m),
                  (0, 0, 0, 0),
                  (-l/(2*I), l/(2*I), -l/(2*I), l/(2*I))))

    # Define LQR weighting matrices
    Q = np.diagflat([150, 150, 15, 15, 0.1, 0.01])  # weight positions more
    R = 1 * np.identity(4)

    # Solve algebraic Riccati equation
    P = linalg.solve_continuous_are(A, B, Q, R)

    # Return LQR cost matrix
    return P

# Calc optimal K around state, using given R matrix and P
def calc_K_matrix(state, P):
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

    # dynamic metrics
    B = np.array(((0, 0, 0, 0),
                  (0, 0, 0, 0),
                  (math.sin(h) / m, math.sin(h) / m, -math.sin(h) / m, -math.sin(h) / m),
                  (-math.cos(h) / m, -math.cos(h) / m, math.cos(h) / m, math.cos(h) / m),
                  (0, 0, 0, 0),
                  (-l/(2*I), l/(2*I), -l/(2*I), l/(2*I))))

    R = 1 * np.identity(4)

    K = (np.linalg.inv(R)).dot(np.transpose(B).dot(P))
    return K