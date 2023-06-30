import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ..helper_classes import SwarmController

import numpy as np
import sys

"""
This is an example of using the SwarmController class to send velocity commands to the swarm.
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
        self.main_loop_timer = self.create_timer(0.1, self.main_loop_cb)
    
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
        if len(self.controller.get_swarming_uris()) > 0:
            for uri in self.controller.get_swarming_uris():
                pos = self.controller.get_position(uri)
                vel = self.controller.get_velocity(uri)
                self.controller.set_velocity(uri, self.turn_to_center(pos, vel, set_speed=1.0))
            self.controller.send_commands()
    
    def turn_to_center(self, pos, vel, height=1.2, turn_scaler=2.5, set_speed=None):
        v_origin = np.array([0.0, 0.0, height]) - np.array(pos)
        a = v_origin - np.dot(v_origin, vel) * np.array(vel) / np.linalg.norm(vel)**2
        a = a / np.linalg.norm(a) * turn_scaler
        new_vel = np.array(vel) + a

        if set_speed is not None:
            new_vel = new_vel / np.linalg.norm(new_vel) * set_speed

        return new_vel
        


def main(args=None):
    rclpy.init(args=args)
    node = PositionCommanderExample()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()