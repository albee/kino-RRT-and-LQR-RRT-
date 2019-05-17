'''
Animate satellite given a trajectory

Hailee Hettrick
'''


import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation, rc
from IPython.display import HTML, Image
import csv

rc('animation', html='html5')

# Read trajectory for a csv file
traj = np.genfromtxt('DT_traj_twofail.csv', delimiter=",")

x = traj[:,1]
y = traj[:,2]
yaw = traj[:,5]
fig = plt.figure()
plt.axis('equal')
plt.grid()
ax = fig.add_subplot(111)
ax.set_xlim(-2, 15)
ax.set_ylim(-2, 13)
plt.plot(x[0],y[0],'bo')
plt.plot(x[-1],y[-1],'rs')
plt.plot(x,y,'k')

patch = patches.Rectangle((0, 0), 0, 0, ec='k',fc='None')

def init():
    ax.add_patch(patch)
    return patch,

def animate(i):
    width = 1.0
    height = width + 0.2
    patch.set_width(width)
    patch.set_height(height)

    x_left = x[i] - (width / 2.0)
    y_lower = y[i] - (height / 2.0)
    x1 = (x_left - x[i])*np.cos(yaw[i]) - (y_lower - y[i])*np.sin(yaw[i]) + x[i]
    y1 = (x_left - x[i])*np.sin(yaw[i]) + (y_lower - y[i])*np.cos(yaw[i]) + y[i]
    patch.set_xy([x1, y1]) 
    patch.angle = np.rad2deg(yaw[i])
    return patch,

anim = animation.FuncAnimation(fig, animate,
                               init_func=init,
                               frames=len(x),
                               interval=100,
                               repeat=False,
                               blit=True)
plt.show()

anim.save('animation.gif', writer='imagemagick', fps=60)
print "done"

