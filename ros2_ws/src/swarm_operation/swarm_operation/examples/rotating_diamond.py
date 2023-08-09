import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ..helper_classes import SwarmController

import time
import sys
import numpy as np

"""
This is an example of using the SwarmController class to send position commands to the swarm.
This swarm is intended to be an OWee demonstration.
"""


class RotatingDiamond(Node):
    def __init__(self):
        super().__init__("rotating_diamond",
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)
        
        self.num_radios = self.get_parameter('number_radios').value

        # create Gui command callback to terminate this node when the GUI closes
        self.GUI_command_sub = self.create_subscription(String, 'GUI_command', self.GUI_command_callback, 10)

        # create swarm controller
        self.controller = SwarmController(self, self.num_radios)

        # start main loop timer at 2 Hz
        self.main_loop_timer = self.create_timer(0.5, self.main_loop_cb)
    
    def GUI_command_callback(self, msg):
        """
        Terminate this node when the GUI sends a terminate command (closing the GUI).
        """
        if msg.data == "terminate/kill all":
            self.destroy_node()
            sys.exit()

    def main_loop_cb(self):
        """
        Main control loop.
        """
        no_drones = len(self.controller.get_swarming_uris())
        if no_drones <= 6:
            grid_points = self.generate_rotating_diamond()
            for i, uri in enumerate(self.controller.get_swarming_uris()):
                self.controller.set_position(uri, grid_points[i])
            self.controller.send_commands()
    
    def generate_rotating_diamond(self):
        """
        Generates vertical diamond in the middle of the room that rotates around z-axis
        """

        center = np.array([0, 0, 1.25])
        max_distance = 0.75
        frequency = 0.05  # Hz
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


def main(args=None):
    rclpy.init(args=args)
    node = RotatingDiamond()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()