'''
Main file for running RRT algorithms. This version provides
the following options, specified in the arguments to rrt( ):

kino : (boolean) : Run a simulation with satellite dynamics, or run a simple RRT dynamics-free.
star : (boolean) : Enable optimimization steps.
show_anim : (boolean) : Show an animation during runtime (only compatible with kino=True)

Keenan Albee, Oliver Jia-Richards, Hailee Hettrick
May-17, 2019
'''
from rrt import *
from plot_auxiliary import *
import Dynamics3DoF as dynamics
import numpy as np
import random

'''
Interesting seeds for the RNG, if you'd like to try them:
16   - does a loop
132  < 0.3
750  < 0.2
1887 < 0.2
2748 < 0.022
3273 < 0.2
8553 < 0.5
8570 < 0.3
8857 < 0.5
8885 < 0.06
 '''
seed = random.randint(1000,10000)
random.seed(2748)

# load dynamics module
my_sat = dynamics.Dynamics3DoF()

# state: [x, y, vx, vy, theta, w]
state_0 = np.asarray([0.25, 0.25, 0., 0., 0.0, 0.])  # initial state
state_g = np.asarray([0., 0., 0., 0., 0.0, 0.])  # goal state

# Define state and control bounds
x_min = -my_sat.pmax; x_max = my_sat.pmax
y_min = -my_sat.pmax; y_max = my_sat.pmax
vx_min = -my_sat.vmax; vx_max = my_sat.vmax
vy_min = -my_sat.vmax; vy_max = my_sat.vmax
h_min = -my_sat.hmax; h_max = my_sat.hmax
w_min = -my_sat.wmax; w_max = my_sat.wmax
bounds = np.asarray([x_min, x_max, y_min, y_max, vx_min, vx_max, vy_min, vy_max, h_min, h_max, w_min, w_max])
ctrl_bounds = np.asarray([0.0, my_sat.Fmax])

### Integration setup

t_step = .1  # seconds
num_dt = 10  # make this an int
dt = t_step/num_dt # seconds
###

### RRT* testing

# Run RRT
V, E, goal_path = rrt(state_0, state_g, bounds, ctrl_bounds, dt, t_step, \
					  kino=True, star=True, show_anim=False)

# Plot path
print "plotting..."
print "seed: ", seed
plot_path_np(V, E, goal_path, bounds)

trajectory = np.empty([0,6])
for i in range(0, len(goal_path.path)):
        start = goal_path.path[i]
        trajectory = np.vstack((trajectory, start))
print trajectory
my_sat.plot_trajectory_passive(trajectory)
###i'