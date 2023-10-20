import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ..helper_classes import SwarmController

import math
import sys
import time

import numpy as np

"""
This is an example of using the SwarmController class to send position commands to the swarm.
The swarm will take the form of a grid which size depends on the size of the swarm.
"""


class MasterCommander(Node):
    def __init__(self):
        super().__init__("master_commander",
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)
        
        self.num_radios = self.get_parameter('number_radios').value

        # create Gui command callback to terminate this node when the GUI closes. and init stored command to pos_controller
        self.GUI_command_sub = self.create_subscription(String, 'GUI_command', self.GUI_command_callback, 10)
        self.GUI_command = String(data="custom/Patterns/activate_pos_commander")
        self.stored_command = None

        # create swarm controller
        self.controller = SwarmController(self, self.num_radios)

        # start main loop timer at 2 Hz
        self.main_loop_timer = self.create_timer(0.5, self.main_loop_cb)
    
    def GUI_command_callback(self, msg):
        """
        Terminate this node when the GUI sends a terminate command (closing the GUI).
        """
        self.GUI_command = msg

        if msg.data == "terminate/kill all":
            self.destroy_node()
            sys.exit()

    def main_loop_cb(self):
        """
        Main control loop.
        """
        # Acquire number of drones
        no_drones = len(self.controller.get_swarming_uris())

        # Define valid control commands
        valid_control_commands = [
                                    "custom/Patterns/activate_pos_commander", 
                                    "custom/Patterns/activate_vel_commander",
                                    "custom/Patterns/activate_rotating_diamond"
                                ]

        if self.GUI_command.data in valid_control_commands:
            self.stored_command = self.GUI_command.data

        if no_drones > 0:
            match self.stored_command:
                # Position Controller #
                case "custom/Patterns/activate_pos_commander":
                    grid_points = self.generate_grid(no_drones)
                    for i, uri in enumerate(self.controller.get_swarming_uris()):
                        self.controller.set_position(uri, grid_points[i])
                    self.controller.send_commands()
                
                # Velocity Controller #
                case "custom/Patterns/activate_vel_commander":
                    for uri in self.controller.get_swarming_uris():
                        pos = self.controller.get_position(uri)
                        vel = self.controller.get_velocity(uri)
                        self.controller.set_velocity(uri, self.turn_to_center(pos, vel, set_speed=1.0))
                    self.controller.send_commands()

                # Rotating Diamond #
                case "custom/Patterns/activate_rotating_diamond":
                    if no_drones <= 6:
                        grid_points = self.generate_rotating_diamond()
                        for i, uri in enumerate(self.controller.get_swarming_uris()):
                            self.controller.set_position(uri, grid_points[i])
                        self.controller.send_commands()

                # Horizontally Rotating Lines #
                case "custom/Patterns/activate_hor_rotating_lines":
                    if no_drones <= 8:
                        grid_points = self.generate_hor_rotating_lines()
                        for i, uri in enumerate(self.controller.get_swarming_uris()):
                            self.controller.set_position(uri, grid_points[i])
                        self.controller.send_commands()

                # Vertically Rotating Lines #
                case "custom/Patterns/activate_ver_rotating_lines":
                    if no_drones <= 8:
                        grid_points = self.generate_ver_rotating_lines()
                        for i, uri in enumerate(self.controller.get_swarming_uris()):
                            self.controller.set_position(uri, grid_points[i])
                        self.controller.send_commands()
                    
    # Utility functions
    def generate_grid(self, no_drones, spacing=0.5, height=1.0, offset=(0.0, 0.0)):
        """
        Create grid points centered around the origin with a given spacing and height.
        """
        grid_size = math.ceil(math.sqrt(no_drones))
        grid = []
        for x in range(grid_size):
            for y in range(grid_size):
                grid.append(((x-(grid_size-1)/2)*spacing+offset[0], (y-(grid_size-1)/2)*spacing+offset[1], height))
        return grid

    def turn_to_center(self, pos, vel, height=1.2, turn_scaler=2.5, set_speed=None):
        v_origin = np.array([0.0, 0.0, height]) - np.array(pos)
        a = v_origin - np.dot(v_origin, vel) * np.array(vel) / np.linalg.norm(vel)**2
        a = a / np.linalg.norm(a) * turn_scaler
        new_vel = np.array(vel) + a

        if set_speed is not None:
            new_vel = new_vel / np.linalg.norm(new_vel) * set_speed

        return new_vel
    
    def generate_rotating_diamond(self):
        """
        Generates vertical diamond in the middle of the room that rotates around z-axis
        """
        center = np.array([0, 0, 1.25])
        max_distance = 0.75
        frequency = 0.1  # Hz
        time_interval = 1.0 / frequency

        top_vertex = center + np.array([0, 0, max_distance])
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
    
    
    def generate_hor_rotating_lines(self):

        R = 1.0
        z_rot = 1.5
        d = 0.7
        x0 = 1.2

        rot_axis = np.array([0, 0, z_rot])

        frequency = 0.05  # Hz
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
    
    def generate_ver_rotating_lines(self):
    #def generate_ver_rotating_lines(self):

        R = 1.0
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


    def smiley(self):
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

def main(args=None):
    rclpy.init(args=args)
    node = MasterCommander()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()