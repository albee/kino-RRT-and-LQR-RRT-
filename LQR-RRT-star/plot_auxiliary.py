'''
RRT plotting functions.
'''

import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np

# Method to plot RRT tree and goal path
def plot_path(V, E, goal_path, bounds):

    # Unpack state bounds
    x_min = bounds[0]
    x_max = bounds[1]
    y_min = bounds[2]
    y_max = bounds[3]
    vx_min = bounds[4]
    vx_max = bounds[5]
    vy_min = bounds[6]
    vy_max = bounds[7]
    h_min  = bounds[8]
    h_max  = bounds[9]
    w_min  = bounds[10]
    w_max  = bounds[11]

    # Initialize figure and axes
    fig, ax = plt.subplots(1, 1, figsize=(10, 7))

    plt.xlim([x_min, x_max])
    plt.ylim([y_min, y_max])

    # plt.xlim([x_min/2.0, x_max/2.0])
    # plt.ylim([y_min/2.0, y_max/2.0])

    plt.grid()
    # ax.set_aspect('equal', adjustable='box')
    ax.axis('equal')

    # Plot tree
    for edge in E:
        start = edge[0]
        end = edge[1]
        ax.plot([start[0], end[0]], [start[1], end[1]], 'k-')

    # Plot goal path
    for i in range(0, len(goal_path.path)-1):
        start = goal_path.path[i]
        end = goal_path.path[i+1]
        ax.plot([start[0], end[0]], [start[1], end[1]], 'r-')

    plt.show()
    plt.pause(100)


def plot_path_np(V, E, goal_path, bounds):
    # Unpack state bounds
    x_min = bounds[0]
    x_max = bounds[1]
    y_min = bounds[2]
    y_max = bounds[3]
    vx_min = bounds[4]
    vx_max = bounds[5]
    vy_min = bounds[6]
    vy_max = bounds[7]
    h_min  = bounds[8]
    h_max  = bounds[9]
    w_min  = bounds[10]
    w_max  = bounds[11]

    # Initialize figure and axes
    fig, ax = plt.subplots(1, 1, figsize=(10, 7))
    print x_min, x_max
    ax.set_xlim(xmin=x_min, xmax=x_max)
    ax.set_ylim(ymin=y_min, ymax=y_max)
    plt.autoscale(False)
    plt.grid()
    # ax.set_aspect('equal', adjustable='box')

    # Plot tree
    for edge in E:
        start_state = edge[0].state
        end_state = edge[1].state
        # print start_state[0:1], end_state[0:1]
        ax.plot([start_state[0], end_state[0]], [start_state[1], end_state[1]], 'k-')

    # Plot goal path
    for i in range(0, len(goal_path.path)-1):
        start_state = goal_path.path[i]
        end_state = goal_path.path[i+1]
        ax.plot([start_state[0], end_state[0]], [start_state[1], end_state[1]], 'r-')

    plt.ion()
    plt.show()
    plt.pause(2)

def plot_near_nodes(near_nodes, fig_num, new_node, rand_state):
    fig = plt.figure(fig_num)
    scat_list = []
    for node in near_nodes:
        scat = plt.scatter(node.state[0], node.state[1], c='y')
        scat_list.append(scat)
    new = plt.scatter(new_node.state[0], new_node.state[1], c='r')
    rand = plt.scatter(rand_state[0], rand_state[1], c='b')
    plt.pause(0.001)
    for scat in scat_list:
        scat.remove()
    new.remove()
    rand.remove()

def plot_path_np_no_goal(E, fig_num):
    fig = plt.figure(fig_num)
    # Plot tree
    for edge in E:
        start_state = edge[0].state
        end_state = edge[1].state
        fig.axes[0].plot([start_state[0], end_state[0]], [start_state[1], end_state[1]], 'g')

def init_plot_path_np_no_goal(V, E, bounds):
    # Unpack state bounds
    x_min = bounds[0]
    x_max = bounds[1]
    y_min = bounds[2]
    y_max = bounds[3]
    vx_min = bounds[4]
    vx_max = bounds[5]
    vy_min = bounds[6]
    vy_max = bounds[7]
    h_min  = bounds[8]
    h_max  = bounds[9]
    w_min  = bounds[10]
    w_max  = bounds[11]

    # Initialize figure and axes
    fig, ax = plt.subplots(1, 1, figsize=(10, 7))

    plt.grid()
    # ax.set_aspect('equal', adjustable='box')

    ax.set_xlim(xmin=x_min, xmax=x_max)
    ax.set_ylim(ymin=y_min, ymax=y_max)
    # plt.axis('equal')
    plt.autoscale(False)

    # Plot tree
    for edge in E:
        start_state = edge[0].state
        end_state = edge[1].state
        ax.plot([start_state[0], end_state[0]], [start_state[1], end_state[1]], 'k-')

    plt.ion()  # make non-blocking
    plt.show()
    plt.pause(0.001)
    fig_num = plt.gcf().number

    return fig_num

def update_plot_path_np_no_goal(last_E, fig_num):
    fig = plt.figure(fig_num)

    # Plot tree
    start_state = last_E[0].state
    end_state = last_E[1].state
    fig.axes[0].plot([start_state[0], end_state[0]], [start_state[1], end_state[1]], 'k-')

    plt.draw()
    plt.pause(0.001)

def rewire_plot_np_no_goal(idx, fig_num, new_node_state, near_node_state):
    fig = plt.figure(fig_num)
    # print(type(fig.axes[0].lines[idx]))
    # fig.axes[0].lines[idx].set_color('r')
    print "E lines ", len(fig.axes[0].lines)

    # fig.axes[0].plot([new_node_state[0], near_node_state[0]], [new_node_state[1], near_node_state[1]], 'g')

    fig.axes[0].lines[idx].set_xdata( [new_node_state[0], near_node_state[0]] )
    fig.axes[0].lines[idx].set_ydata( [new_node_state[1], near_node_state[1]] )
    fig.canvas.draw()
    # fig.axes[0].lines[idx].remove()
    print 'removed!!'
    plt.draw()
    plt.pause(0.001)

# moviewriter setup
def setup_movie_writer(fps):
    FFMpegWriter = anim.writers['ffmpeg']
    metadata = dict(title='Movie Test', artist='Matplotlib',
        comment='Movie support!')
    writer = FFMpegWriter(fps=fps, metadata=metadata)
    return writer