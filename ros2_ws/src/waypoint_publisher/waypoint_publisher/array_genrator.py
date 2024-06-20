import numpy as np
from matplotlib import pyplot as plt

def generate_waypoints(timesteps, amplitude, landing_cycles):
    points = np.array([[1, 1, 0], [1, -1, 0], [-1, -1, 0], [-1, 1, 0]])
    num_points = len(points)
    idx = 1
    # Total segments will be equal to the number of waypoints - 1
    segments = timesteps - 1
    
    # Calculate waypoints
    waypoints = []
    
    for i in range(num_points):
        start = points[i]
        end = points[(i + 1) % num_points]
        
        for t in range(segments // num_points + 1):
            alpha = t / (segments // num_points)
            x = (1 - alpha) * start[0] + alpha * end[0]
            y = (1 - alpha) * start[1] + alpha * end[1]
            z = amplitude*np.sin(np.pi * alpha)  # Sinusoidal Z coordinate
            
            waypoints.append([x, y, z, idx])
            idx += 1

        for _ in range(landing_cycles):
            waypoints.append([-1, -1, -1, idx])
            idx += 1
    
    return np.array(waypoints)  # Make sure we have exactly `timesteps` waypoints

# Example usage:
timesteps = 100
amplitude = 0.3
landing_cycles = 5
waypoints = generate_waypoints(timesteps, amplitude, landing_cycles)

# fig = plt.figure(figsize=(12, 12))
# ax = fig.add_subplot(projection='3d')

# cmhot = plt.get_cmap("hot")

# ax.scatter(waypoints[:,0], waypoints[:,1], waypoints[:,2], c=waypoints[:,3], cmap=cmhot)
# plt.axis('equal')
# plt.show()

print(waypoints)