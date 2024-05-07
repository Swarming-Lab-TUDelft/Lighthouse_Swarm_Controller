import math
import time
import numpy as np

# Utility functions
def generate_grid(no_drones, spacing=0.5, height=1.0, offset=(0.0, 0.0)):
    """
    Create grid points centered around the origin with a given spacing and height.
    """
    grid_size = math.ceil(math.sqrt(no_drones))
    grid = []
    for x in range(grid_size):
        for y in range(grid_size):
            grid.append(((x-(grid_size-1)/2)*spacing+offset[0], (y-(grid_size-1)/2)*spacing+offset[1], height))
    return grid

def generate_velocities(pos, vel, height=1.2, turn_scaler=2.5, set_speed=None):
    v_origin = np.array([0.0, 0.0, height]) - np.array(pos)
    a = v_origin - np.dot(v_origin, vel) * np.array(vel) / np.linalg.norm(vel)**2
    a = a / np.linalg.norm(a) * turn_scaler
    new_vel = np.array(vel) + a

    if set_speed is not None:
        new_vel = new_vel / np.linalg.norm(new_vel) * set_speed

    return new_vel

def generate_rotating_diamond():
    """
    Generates vertical diamond in the middle of the room that rotates around z-axis
    """
    center = np.array([0, 0, 1.25])
    max_distance = 0.75
    frequency = 0.1  # Hz
    time_interval = 1.0 / frequency

    top_vertex = center + np.array([0, 0, max_distance])
    middle_vertex = center + np.array([0.05, 0, 0])
    bottom_vertex = center - np.array([0, 0, max_distance])
    left_vertex = center - np.array([max_distance, 0, 0])
    right_vertex = center + np.array([max_distance, 0, 0])
    front_vertex = center + np.array([0, max_distance, 0])
    back_vertex = center - np.array([0, max_distance, 0])

    vertices = np.array([
        top_vertex, bottom_vertex,
        left_vertex, right_vertex,
        front_vertex, back_vertex
    ])

    t = time.time()
    angle = 2 * np.pi * (t % time_interval) / time_interval
        
    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])
    rotated_vertices = np.dot(vertices, rotation_matrix.T)
    return rotated_vertices


def generate_hor_rotating_lines():

    R = 0.8
    z_rot = 1.5
    d = 0.7
    x0 = 1.2

    rot_axis = np.array([0, 0, z_rot])

    frequency = 0.1  # Hz
    time_interval = 1.0 / frequency
    

    A1 = np.array([x0 - 0.0*d,       0, z_rot + R])
    A2 = np.array([x0 - 1.0*d,       0, z_rot + R])
    A3 = np.array([x0 - 2.0*d,       0, z_rot + R])
    A4 = np.array([x0 - 3.0*d,       0, z_rot + R])

    B1 = np.array([x0 - 0.5*d,       0, z_rot - R])
    B2 = np.array([x0 - 1.5*d,       0, z_rot - R])
    B3 = np.array([x0 - 2.5*d,       0, z_rot - R])
    B4 = np.array([x0 - 3.5*d,       0, z_rot - R])


    vertices = np.array([
        A1, A2, A3, A4, B1, B2, B3, B4
    ]) 


    # Translate first
    vertices = vertices - rot_axis


    # Rotate
    t = time.time()
    angle = 2 * np.pi * (t % time_interval) / time_interval

    rotation_matrix = np.array([
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)]
    ])

    vertices = np.dot(vertices, rotation_matrix.T)


    # Translate back
    vertices = vertices + rot_axis
    

    return vertices

def generate_ver_rotating_lines():

    R = 0.8
    d = 0.7
    z0 = 0.3

    frequency = 0.05  # Hz
    time_interval = 1.0 / frequency
    

    A1 = np.array([R,       0, z0 + 0*d])
    A2 = np.array([R,       0.1, z0 + 1*d])
    A3 = np.array([R,       -0.1, z0 + 2*d])
    A4 = np.array([R,       0, z0 + 3*d])

    B1 = np.array([-R,       0, z0 + 0*d])
    B2 = np.array([-R,       0.1, z0 + 1*d])
    B3 = np.array([-R,       -0.1, z0 + 2*d])
    B4 = np.array([-R,       0, z0 + 3*d])


    vertices = np.array([
        A1, A2, A3, A4, B1, B2, B3, B4
    ]) 

    


    # Rotate
    t = time.time()
    angle = 2 * np.pi * (t % time_interval) / time_interval

    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])

    vertices = np.dot(vertices, rotation_matrix.T)
    
    return vertices

def generate_spiral():

    R = 0.5
    z0 = 0.25

    frequency = 0.2  # Hz
    time_interval = 1.0 / frequency
    
    dth = 60 / 57.3
    dh = 0.3


    A = np.array([R * np.cos(0*dth), R * np.sin(0*dth), 0*dh + z0])
    B = np.array([R * np.cos(1*dth), R * np.sin(1*dth), 1*dh + z0])
    C = np.array([R * np.cos(2*dth), R * np.sin(2*dth), 2*dh + z0])
    D = np.array([R * np.cos(3*dth), R * np.sin(3*dth), 3*dh + z0])
    E = np.array([R * np.cos(4*dth), R * np.sin(4*dth), 4*dh + z0])
    F = np.array([R * np.cos(5*dth), R * np.sin(5*dth), 5*dh + z0])
    G = np.array([R * np.cos(6*dth), R * np.sin(6*dth), 6*dh + z0])
    H = np.array([R * np.cos(7*dth), R * np.sin(7*dth), 7*dh + z0])
    I = np.array([R * np.cos(8*dth), R * np.sin(8*dth), 9*dh + z0])
    


    vertices = np.array([
        A, B, C, D, E, F, G, H, I
    ]) 


    # Rotate
    t = time.time()
    angle = 2 * np.pi * (t % time_interval) / time_interval

    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])

    vertices = np.dot(vertices, rotation_matrix.T)
    
    return vertices

def generate_smiley():
    """
    Generates smiley in the middle of the room that rotates around z-axis
    """
    frequency = 0.05  # Hz
    time_interval = 1.0 / frequency

    left_eye = np.array([0, 0.5, 1.5])
    right_eye = np.array([0, -0.5, 1.5])

    mouth1 = np.array([0, -0.8, 1.0])
    mouth2 = np.array([0, -0.5, 0.6])
    mouth3 = np.array([0, -0.2, 0.4])
    mouth4 = np.array([0, +0.2, 0.4])
    mouth5 = np.array([0, +0.5, 0.6])
    mouth6 = np.array([0, +0.8, 1.0])


    vertices = np.array([
        left_eye, right_eye, mouth1, mouth2, mouth3, mouth4, mouth5, mouth6
    ])

    t = time.time()
    angle = 2 * np.pi * (t % time_interval) / time_interval
        
    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])
    # rotated_vertices = np.dot(vertices, rotation_matrix.T)
    return vertices

def generate_sinwave():  
    ## Define the grid parameters
    x_min = -1.0  # Minimum x-coordinate
    x_max = 1.0   # Maximum x-coordinate
    num_drones_per_line = 4  # Number of points in the x direction
    num_lines = 2  # Number of lines

    # Generate x-coordinates for the grid
    x_coordinates = np.linspace(x_min, x_max, num_drones_per_line)

    # Create an empty array to store the grid points
    grid_points = []

    # Define the amplitude and frequency of the sine wave for bobbing
    amplitude = 0.4  # Adjust as needed
    frequency = 0.8  # Adjust as needed

    # Create the two lines at z=1.0
    for line_index in range(num_lines):
        # Calculate the y-coordinate for the current line
        y = (line_index - (num_lines - 1) / 2) * 0.75  # Adjust the spacing as needed

        # Apply sine wave to z coordinate with a phase shift based on time
        z = 1.0 + amplitude * np.sin(frequency * time.time() - x_coordinates)

        # Create the points for the current line
        line_points = np.column_stack((x_coordinates, np.full(num_drones_per_line, y), np.full(num_drones_per_line, z)))
        
        # Append the line points to the grid_points array
        grid_points.append(line_points)

    # Stack the line points vertically to create the final grid
    grid_points = np.vstack(grid_points)

    return grid_points 
    # # Define the number of drones in each line and their spacing
    # num_drones = 8
    # spacing = 3.0 / (num_drones - 1)  # Total length is 3.0 (1.5 on each side)

    # # Define the rotation angle in radians
    # rotation_angle = np.radians(45)  # 45 degrees

    # # Initialize an empty NumPy array to store the waypoints
    # waypoints = np.empty((2 * num_drones, 3), dtype=np.float64)

    # # Define the amplitude and frequency of the sine wave for bobbing
    # amplitude = 0.5  # Adjust as needed
    # frequency = 0.2  # Adjust as needed

    # # Create waypoints for each drone in the first line
    # for i in range(int(num_drones/2)):
    #     x = -1.5 + i * spacing  # Distribute drones between x=-1.5 and x=1.5
    #     y = 0.0

    #     # Apply sine wave to z coordinate with a phase shift based on time
    #     z = 1.0 + amplitude * np.sin(2 * np.pi * frequency * x - 2 * np.pi * frequency * time.time())
        
    #     waypoint = np.array([x, y, z])
    #     waypoints[i] = waypoint

    # # Create waypoints for each drone in the second line, offset to the side
    # for i in range(int(num_drones/2)):
    #     i = i + int(num_drones/2)
    #     x = -1.5 + i * spacing  # Distribute drones between x=-1.5 and x=1.5
    #     y = 2.0  # Offset the second line vertically
        
    #     # Apply sine wave to z coordinate with a phase shift based on time
    #     z = 1.0 + amplitude * np.sin(2 * np.pi * frequency * x - 2 * np.pi * frequency * time.time())
    
    #     waypoint = np.array([x, y, z])
    #     waypoints[i] = waypoint

    # return waypoints

def generate_grasshopper():
    frequency = 0.5
    time_interval = 1/frequency
    t = time.time()

    angle = 2 * np.pi * (t%time_interval) / time_interval
    idx = (t%time_interval) / time_interval
    segment = (t%(time_interval*8)) / (time_interval*8)

    if segment < 0.125:
        x = 1.0
        y = -1.0 + (1.0 - -1.0) * idx
        height = np.abs(np.sin(angle*0.5))

    elif segment < 0.25:
        x = 1.0
        y = 1.0
        height = 0

    elif segment < 0.375:
        x = 1.0 + (-1.0 - 1.0) * idx
        y = 1.0
        height = np.abs(np.sin(angle*0.5))

    elif segment < 0.5:
        x = -1.0
        y = 1.0
        height = 0

    elif segment < 0.625:
        x = -1.0
        y = 1.0 + (-1.0 - 1.0) * idx
        height = np.abs(np.sin(angle*0.5))

    elif segment < 0.75:
        x = -1.0
        y = -1.0
        height = 0

    elif segment < 0.875:
        x = -1.0 + (1.0 - -1.0) * idx
        y = -1.0 
        height = np.abs(np.sin(angle*0.5))

    else:
        x = 1.0
        y = -1.0
        height = 0

    point1 = [x, y, height]

    #follower
    t = t - 0.6
    angle = 2 * np.pi * (t%time_interval) / time_interval
    idx = (t%time_interval) / time_interval
    segment = (t%(time_interval*8)) / (time_interval*8)

    if segment < 0.125:
        x = 1.0
        y = -1.0 + (1.0 - -1.0) * idx
        height = np.abs(np.sin(angle*0.5))

    elif segment < 0.25:
        x = 1.0
        y = 1.0
        height = 0

    elif segment < 0.375:
        x = 1.0 + (-1.0 - 1.0) * idx
        y = 1.0
        height = np.abs(np.sin(angle*0.5))

    elif segment < 0.5:
        x = -1.0
        y = 1.0
        height = 0

    elif segment < 0.625:
        x = -1.0
        y = 1.0 + (-1.0 - 1.0) * idx
        height = np.abs(np.sin(angle*0.5))

    elif segment < 0.75:
        x = -1.0
        y = -1.0
        height = 0

    elif segment < 0.875:
        x = -1.0 + (1.0 - -1.0) * idx
        y = -1.0 
        height = np.abs(np.sin(angle*0.5))

    else:
        x = 1.0
        y = -1.0
        height = 0

    point2 = [x+0.3, y, height]
    point3 = [x+0.3, y+0.3, height]
    point4 = [x, y+0.3, height]

    vertices = np.array([point1, point2, point3. point4])
    return vertices

