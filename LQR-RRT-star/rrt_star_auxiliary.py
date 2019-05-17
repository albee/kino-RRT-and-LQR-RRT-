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


# Method to calculate nearest node to randomly generated state in existing tree
def nearest(V, rand_state):

    # Unpack randomly generated state
    x_rand  = rand_state[0]
    y_rand  = rand_state[1]

    # Initialize variable to record shortest distance
    d_min = 0

    # Initialize variable to record closest node
    nearest_node = []

    # Iterate through all nodes and determine the closest
    for node in V:
        # Unpack node state
        state = node.state
        x_node  = state[0]
        y_node  = state[1]

        # Calculate distance to randomly generated node
        d = math.sqrt((x_rand-x_node)**2 + (y_rand-y_node)**2)  # L2-norm

        # Store if shorter than the shortest distance
        if d < d_min or d_min == 0:
            d_min = d
            nearest_node = node

    # Return nearest node
    return nearest_node


# Method to steer robot in direction of the randomly generated state
def steer(nearest_node, rand_state, d_max):
    # Unpack nearest node and random state
    x_near = nearest_node.state[0]
    y_near = nearest_node.state[1]
    x_rand = rand_state[0]
    y_rand = rand_state[1]

    # Determine vector between nearest node and random state
    dx = x_rand - x_near
    dy = y_rand - y_near
    d = math.sqrt(dx**2 + dy**2)

    # If distance is longer than maximum movement distance, shorten to d_max
    if d > d_max:
        dx = dx/d*d_max
        dy = dy/d*d_max
        d = d_max

    # Calculate new node position
    x_new = x_near + dx
    y_new = y_near + dy
    state = np.array([x_new, y_new])

    # Calculate new node total cost-to-come
    cost = nearest_node.cost + d

    # Create new node
    new_node = SearchNode(state, nearest_node, cost)

    # Return new node
    return new_node

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


def rewire_from_x_new(near_nodes, new_node, E, fig_num):
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

            # remove from the plot
            rewire_plot_np_no_goal(idx, fig_num, new_node.state, near_node.state)


# Compute the ball radius of near nodes to check
def get_ball_radius(card_V, state_dimension, volume_obstacle_free, max_dist):
    d = state_dimension
    v = volume_obstacle_free
    if d == 2:
        v_unit_ball = math.pi  # volume of unit ball
    else:
        raise NameError('not yet implemented')

    gamma_star = 2*(1 + 1/d)**(1/d) * (v/v_unit_ball)**1/d
    gamma = gamma_star*10.0 # must be larger than gamma*, provided by Karaman
    return min(gamma * math.log(card_V)/card_V, max_dist)

# Straight-line path from node-to-node. Should be augmented to cost-to-go.
def line(from_node, to_node):
    state_from = from_node.state
    state_to = to_node.state
    cost_to_go = np.linalg.norm(state_from - state_to)
    # print "cost to go", cost_to_go
    return cost_to_go

# Find nodes that are within radius of new_node
def near(V, E, new_node, radius):
    near_nodes = []
    new_node_state = new_node.state

    for node in V:
        node_state = node.state

        # Calculate distance to randomly generated node
        dist = np.linalg.norm(new_node_state - node_state)  # L2-norm

         # Store if shorter than radius
        if dist < radius:
            near_nodes.append(node)

    return near_nodes

def collision_free(node_from, node_to):
    return True

# equivalent to obstacle_free from [Karaman and Frazzoli, 2011]
def in_bounds(state, bounds):

    # Unpack state
    x = state[0]
    y = state[1]

    # Unpack bounds
    x_min = bounds[0]
    x_max = bounds[1]
    y_min = bounds[2]
    y_max = bounds[3]

    # Check bounds
    if x < x_min or x > x_max:
        return False

    if y < y_min or y > y_max:
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
    else:
        raise NameError('not yet implemented dude')