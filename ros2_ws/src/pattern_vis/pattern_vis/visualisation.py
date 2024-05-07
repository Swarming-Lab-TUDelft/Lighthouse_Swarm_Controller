#!/usr/bin/env python3

import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from itertools import count
from matplotlib.animation import FuncAnimation

from swarm_operation.examples.waypoint_functions import *

def visualize_dynamic_3d_points():
    no_drones = 8
    # Select the waypoint function as listed in the swarm_operation examples
    # Options are:
    # gen = generate_rotating_diamond
    # gen = generate_hor_rotating_lines
    # gen = generate_ver_rotating_lines
    # gen = generate_spiral
    # gen = generate_smiley
    # gen = generate_sinwave
    gen = generate_grasshopper

    # Initialize the figure and 3D axes
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Set to size of drone cage (ABS bounds)
    ax.set_xlim((-1.5, 1.75))
    ax.set_ylim((-1.84, 1.7))
    ax.set_zlim((0.0, 2.5))

    # Create labeled axes
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Create an empty scatter plot
    sc = ax.scatter([], [], [], c='b', marker='o')

    # Initialize the data and iterator
    data = np.empty((0, 3))
    counter = count()

    # Function to update the scatter plot data
    def update(frame):
        nonlocal data
        nonlocal sc

        points = gen()
        
        sc.remove()
        sc = ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', marker='o')

    # Create an animation
    ani = FuncAnimation(fig, update, blit=False, interval=10)

    # Show the plot
    plt.show()

if __name__ == '__main__':
    visualize_dynamic_3d_points()