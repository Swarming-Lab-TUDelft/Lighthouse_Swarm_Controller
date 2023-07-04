import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ..helper_classes import SwarmController

import math
import sys

"""
This is an example of using the SwarmController class to send position commands to the swarm.
The swarm will take the form of a grid which size depends on the size of the swarm.
"""


class PositionCommanderExample(Node):
    def __init__(self):
        super().__init__("position_commander",
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
        if no_drones > 0:
            grid_points = self.generate_grid(no_drones)
            for i, uri in enumerate(self.controller.get_swarming_uris()):
                self.controller.set_position(uri, grid_points[i])
            self.controller.send_commands()
    
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


def main(args=None):
    rclpy.init(args=args)
    node = PositionCommanderExample()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()