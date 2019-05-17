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
    state = (x_new, y_new)

    # Calculate new node cost
    cost = nearest_node.cost + d

    # Create new node
    new_node = SearchNode(state, nearest_node, cost)

    # Return new node
    return new_node

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