import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from helper_classes import SwarmController

import math


class PositionCommanderExample(Node):
    def __init__(self):
        super().__init__("position_commander",
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        self.num_radios = self.get_parameter('number_radios').value

        self.controller = SwarmController(self, self.num_radios)

        self.main_loop_timer = self.create_timer(0.1, self.main_loop_cb)
    
    def main_loop_cb(self):
        no_drones = len(self.controller.get_swarming_uris())
        if no_drones > 0:
            grid_points = self.create_grid(no_drones)
            for i, uri in enumerate(self.controller.get_swarming_uris()):
                self.controller.set_position(uri, grid_points[i])
            self.controller.send_commands()
    
    def create_grid(self, no_drones, spacing=0.5, height=1.0):
        """
        Create grid points centered around the origin with a given spacing and height.
        """
        grid_size = math.ceil(math.sqrt(no_drones))
        grid = []
        for x in range(grid_size):
            for y in range(grid_size):
                grid.append(((x-(grid_size-1)/2)*spacing, (y-(grid_size-1)/2)*spacing, height))
        return grid



def main(args=None):
    rclpy.init(args=args)
    node = PositionCommanderExample()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()