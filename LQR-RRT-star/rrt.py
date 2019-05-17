'''
(kino-)RRT(*) loop

Oliver Jia-Richards, Keenan Albee, Hailee Hettrick
'''

import random
from tree_auxiliary import *
import rrt_auxiliary as kino_rrt_aux
import lqr_rrt_star_auxiliary as lqr_rrt_star_aux
import numpy as np
import math
from plot_auxiliary import *

'''
Mandatory function call to rrt in order to access other functions in this file
'''
def rrt(initial_state, goal_state, bounds, ctrl_bounds, dt, t_step, **kwargs):
    # Parse keyword arguments
    rrt_type = 'guided-kino'  # run kino-rrt
    show_anim = False
    for key, value in kwargs.items():
        if key == "rrt_type":
            rrt_type = value
            print "using : ", rrt_type
        if key == "show_anim":
            show_anim = value

    if rrt_type == "guided-kino":
        V, E, goal_path = guided_kino_rrt(initial_state, goal_state, bounds, ctrl_bounds, dt, t_step, show_anim)
    elif rrt_type == "lqr-rrt-star":
        V, E, goal_path = lqr_rrt_star(initial_state, goal_state, bounds, ctrl_bounds, dt, t_step, show_anim)
    elif rrt_type == "kino-euclidean":
        V, E, goal_path = kino_rrt_euclidean(initial_state, goal_state, bounds, ctrl_bounds, dt, t_step, show_anim)
    else:
        raise Exception("please input a valid algorithm")
    # Return nodes, edges, and goal path
    return V, E, goal_path


'''
Kino-RRT, with motion primitive steering and an LQR cost-to-go distance metric.
'''
def guided_kino_rrt(initial_state, goal_state, bounds, ctrl_bounds, dt, t_step, show_anim):
    # Unpacking
    x_min = bounds[0]; x_max = bounds[1]; y_min = bounds[2]; y_max = bounds[3]
    vx_min = bounds[4]; vx_max = bounds[5]; vy_min = bounds[6]; vy_max = bounds[7]
    h_min  = bounds[8]; h_max  = bounds[9]; w_min  = bounds[10]; w_max  = bounds[11]

    x_goal = goal_state[0]; y_goal = goal_state[1]
    vx_goal = goal_state[2]; vy_goal = goal_state[3]
    h_goal  = goal_state[4]; w_goal  = goal_state[5]

    # Parameters
    max_count = 500
    goal_bias_count = 10
    max_dist = float('inf')  # this is currently not restricted
    goal_tol = .01  # goal tolerance

    start_node = SearchNode(initial_state)

    # Initialize lists of nodes and edges, goal path, distance
    state_dimension = initial_state.size
    volume_obstacle_free = lqr_rrt_star_aux.volume_obstacle_free(bounds, state_dimension)

    # Setup
    fig_num = 0
    V = [start_node]
    E = []
    goal_path = None
    goal_distance = float('inf')
    lowest_distance = lqr_rrt_star_aux.lqr_distance(initial_state, goal_state)

    if show_anim == True:
        print "animating..."
        fig_num = init_plot_path_np_no_goal(V, E, bounds)

    # Track best node
    lowest_distance = lqr_rrt_star_aux.lqr_distance(initial_state, goal_state)
    best_node = start_node

    # RRT loop: sample until path to goal is found
    counter = 1
    sample_counter = 1
    while counter <= max_count and goal_distance > goal_tol:
        # print "Iteration number: ", counter
        if sample_counter == goal_bias_count:

            x_rand  = x_goal
            y_rand  = y_goal
            vx_rand = vx_goal
            vy_rand = vy_goal
            h_rand  = h_goal
            w_rand  = w_goal

            sample_counter = 1

        else:
            x_rand  = random.uniform(x_min, x_max)
            y_rand  = random.uniform(y_min, y_max)
            vx_rand = random.uniform(vx_min, vx_max)
            vy_rand = random.uniform(vy_min, vy_max)
            h_rand  = random.uniform(h_min, h_max)
            w_rand  = random.uniform(w_min, w_max)

            sample_counter += 1

        rand_state = (x_rand, y_rand, vx_rand, vy_rand, h_rand, w_rand)

        # Determine nearest node: nearest()
        nearest_node = lqr_rrt_star_aux.LQR_nearest(V, rand_state)

        # Steer towards nearest node using guided steer()
        new_node = lqr_rrt_star_aux.guided_steer(nearest_node, rand_state, max_dist, ctrl_bounds, dt, t_step, bounds)

        # Check if new node is in bounds: obstacle_free()
        if not lqr_rrt_star_aux.in_bounds(new_node.state, bounds):
            # print "out of bounds!"
            continue

        ### kino-RRT
        V.append(new_node)
        E.append([nearest_node, new_node])
        ### kino-RRT

        # This code block enables RRT*. It is currently disabled because the steering
        # function does not guarantee getting to the goal exactly, so rewires may be inaccurate.
        ### RRT*
        # else:
        #     radius = lqr_rrt_star_aux.get_ball_radius( len(V), state_dimension, volume_obstacle_free, max_dist )
        #     near_nodes = lqr_rrt_star_aux.LQR_near(V, E, new_node, radius)
        #     V.append(new_node)
        #     if show_anim == True:
        #         print "near nodes:", len(near_nodes)
        #         plot_near_nodes(near_nodes, fig_num, new_node, rand_state)

        #     # Choose a parent for x_new: choose_parent()
        #     lqr_rrt_star_aux.wire_to_x_new(near_nodes, new_node, nearest_node, E)

        #     # Attempt to rewire x_near: rewire()
        #     lqr_rrt_star_aux.rewire_from_x_new(near_nodes, new_node, E, fig_num, show_anim)
        ### RRT*

        if show_anim == True:
            update_plot_path_np_no_goal(E[-1], fig_num)

        ### Bookkeeping
        counter += 1
        goal_distance = lqr_rrt_star_aux.lqr_distance(new_node.state, goal_state)

        if goal_distance < lowest_distance:
            lowest_distance = goal_distance
            best_node = new_node
            # print "closest node to state_g so far: ", lowest_distance

    print("max count or goal_tol reached...")
    goal_path = Path(best_node)

    return V, E, goal_path


'''
LQR-RRT*, applied to 3DoF underactuated spacecraft dynamics.
Essentially, merges rrt_star() and kino_rrt() with modifications
to rrt_star() for dynamical constraints (e.g. new cost-to-go metric).
'''
def lqr_rrt_star(initial_state, goal_state, bounds, ctrl_bounds, dt, t_step, show_anim):
    # Unpacking
    x_min = bounds[0]; x_max = bounds[1]; y_min = bounds[2]; y_max = bounds[3]
    vx_min = bounds[4]; vx_max = bounds[5]; vy_min = bounds[6]; vy_max = bounds[7]
    h_min  = bounds[8]; h_max  = bounds[9]; w_min  = bounds[10]; w_max  = bounds[11]

    x_goal = goal_state[0]; y_goal = goal_state[1]
    vx_goal = goal_state[2]; vy_goal = goal_state[3]
    h_goal  = goal_state[4]; w_goal  = goal_state[5]

    # Parameters
    record = False
    max_count = 500
    goal_bias_count = 10
    max_dist = float('inf')  # this is currently not restricted
    goal_tol = .01  # goal tolerance

    start_node = SearchNode(initial_state)

    # Initialize lists of nodes and edges, goal path, distance
    state_dimension = initial_state.size
    volume_obstacle_free = lqr_rrt_star_aux.volume_obstacle_free(bounds, state_dimension)

    # Setup
    fig_num = 0
    V = [start_node]
    E = []
    goal_path = None
    goal_distance = float('inf')
    lowest_distance = lqr_rrt_star_aux.lqr_distance(initial_state, goal_state)

    if show_anim == True:
        fig_num = init_plot_path_np_no_goal(V, E, bounds)

    # Track best node
    lowest_distance = lqr_rrt_star_aux.lqr_distance(initial_state, goal_state)
    best_node = start_node

    # RRT loop: sample until path to goal is found
    counter = 1
    sample_counter = 1
    while counter <= max_count and goal_distance > goal_tol:
        # print "Iteration number: ", counter
        if sample_counter == goal_bias_count:
            x_rand  = x_goal
            y_rand  = y_goal
            vx_rand = vx_goal
            vy_rand = vy_goal
            h_rand  = h_goal
            w_rand  = w_goal

            sample_counter = 1

        else:
            x_rand  = random.uniform(x_min, x_max)
            y_rand  = random.uniform(y_min, y_max)
            vx_rand = random.uniform(vx_min, vx_max)
            vy_rand = random.uniform(vy_min, vy_max)
            h_rand  = random.uniform(h_min, h_max)
            w_rand  = random.uniform(w_min, w_max)

            sample_counter += 1

        rand_state = (x_rand, y_rand, vx_rand, vy_rand, h_rand, w_rand)

        # Determine nearest node: nearest()
        nearest_node = lqr_rrt_star_aux.LQR_nearest(V, rand_state)

        # Steer towards nearest node using LQR policy: steer()
        new_node = lqr_rrt_star_aux.LQR_steer(nearest_node, rand_state, max_dist, ctrl_bounds, dt, t_step, bounds)

        # Check if new node is in bounds: obstacle_free()
        if not lqr_rrt_star_aux.in_bounds(new_node.state, bounds):
            # print "out of bounds!"
            continue
        else:
            radius = lqr_rrt_star_aux.get_ball_radius( len(V), state_dimension, volume_obstacle_free, max_dist )
            near_nodes = lqr_rrt_star_aux.LQR_near(V, E, new_node, radius)
            V.append(new_node)
            if show_anim == True:
                print "near nodes:", len(near_nodes)
                plot_near_nodes(near_nodes, fig_num, new_node, rand_state)

            # Choose a parent for x_new: choose_parent()
            lqr_rrt_star_aux.wire_to_x_new(near_nodes, new_node, nearest_node, E)

            # Attempt to rewire x_near: rewire()
            lqr_rrt_star_aux.rewire_from_x_new(near_nodes, new_node, E, fig_num, show_anim)

        if show_anim == True:
            update_plot_path_np_no_goal(E[-1], fig_num)

        ### Bookkeeping
        counter += 1
        goal_distance = lqr_rrt_star_aux.lqr_distance(new_node.state, goal_state)

        if goal_distance < lowest_distance:
            lowest_distance = goal_distance
            best_node = new_node
            # print "closest node to state_g so far: ", lowest_distance

        if record and counter%10 == True:
            writer.grab_frame()

    print("max count reached or goal_tol...")
    goal_path = Path(best_node)

    return V, E, goal_path

'''
kino-RRT-Euclidean
RRT, with forward evaluation of 3DoF underactuated spacecraft dynamics.
Uses a Euclidean distance metric, which isn't very effective.
'''
def kino_rrt_euclidean(initial_state, goal_state, bounds, ctrl_bounds, dt, t_step, show_anim):
    # Unpacking
    x_min = bounds[0]; x_max = bounds[1]; y_min = bounds[2]; y_max = bounds[3]
    vx_min = bounds[4]; vx_max = bounds[5]; vy_min = bounds[6]; vy_max = bounds[7]
    h_min  = bounds[8]; h_max  = bounds[9]; w_min  = bounds[10]; w_max  = bounds[11]

    x_goal = goal_state[0]; y_goal = goal_state[1]
    vx_goal = goal_state[2]; vy_goal = goal_state[3]
    h_goal  = goal_state[4]; w_goal  = goal_state[5]

    # Parameters
    record = False
    max_count = 1000
    goal_bias_count = 10
    max_dist = float('inf')  # this is currently not restricted
    goal_tol = .02  # goal tolerance

    # Track best node
    start_node = SearchNode(initial_state)
    best_node = start_node
    lowest_distance = lqr_rrt_star_aux.L2_distance(initial_state, goal_state)

    # Initialize lists of nodes and edges, goal path, distance
    state_dimension = initial_state.size

    # Setup
    fig_num = 0
    V = [start_node]
    E = []
    goal_path = None
    goal_distance = float('inf')

    if show_anim == True:
        fig_num = init_plot_path_np_no_goal(V, E, bounds)

    # RRT loop: sample until path to goal is found
    counter = 1
    sample_counter = 1
    # writer = setup_movie_writer(8)
    # with writer.saving(plt.figure(fig_num), "blargh.mp4", 400):
    while counter <= max_count and goal_distance > goal_tol:
        # print "Iteration number: ", counter
        if sample_counter == goal_bias_count:
            x_rand  = x_goal
            y_rand  = y_goal
            vx_rand = vx_goal
            vy_rand = vy_goal
            h_rand  = h_goal
            w_rand  = w_goal

            sample_counter = 1

        else:
            x_rand  = random.uniform(x_min, x_max)
            y_rand  = random.uniform(y_min, y_max)
            vx_rand = random.uniform(vx_min, vx_max)
            vy_rand = random.uniform(vy_min, vy_max)
            h_rand  = random.uniform(h_min, h_max)
            w_rand  = random.uniform(w_min, w_max)
            rand_state = np.array([x_rand, y_rand, vx_rand, vy_rand, h_rand, w_rand])

            sample_counter += 1

        # Determine nearest node: nearest()
        nearest_node = lqr_rrt_star_aux.L2_nearest(V, rand_state)

        # Steer towards nearest node using random steering: steer()
        new_node = lqr_rrt_star_aux.random_steer(nearest_node, rand_state, max_dist, ctrl_bounds, dt, t_step, bounds)

        # Check if new node is in bounds: obstacle_free()
        if not lqr_rrt_star_aux.in_bounds(new_node.state, bounds):
            continue

        V.append(new_node)
        E.append([nearest_node, new_node])

        if show_anim == True:
            update_plot_path_np_no_goal(E[-1], fig_num)

        ### Bookkeeping
        counter += 1
        goal_distance = lqr_rrt_star_aux.L2_distance(new_node.state, goal_state)

        if goal_distance < lowest_distance:
            lowest_distance = goal_distance
            best_node = new_node
            print "closest node to state_g so far: ", lowest_distance

    print("max count reached...")
    goal_path = Path(best_node)

    return V, E, goal_path

