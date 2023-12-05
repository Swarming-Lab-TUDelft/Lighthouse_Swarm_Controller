import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sys

from ..helper_classes import SwarmController
from .waypoint_functions import *

"""
This is an example of using the SwarmController class to send position commands to the swarm.
The swarm will take the form of a grid which size depends on the size of the swarm.
"""

# Define custom swarm commands/patterns
# Format: "header": (("button text", "command"), ...)
# command will be sent to the topic GUI_command as "custom/'header'/'command' "
custom_swarm_commands = {
    "Patterns": (
        ("Velocity", "activate_vel_commander"),
        ("Position", "activate_pos_commander"),
        ("Diamond", "activate_rotating_diamond"),
        ("H. Lines", "activate_hor_rotating_lines"),
        ("V. Lines", "activate_ver_rotating_lines"),
        ("Sin Wave", "activate_sin_wave"),
    )
}

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

        # Flatten control commands
        valid_control_commands = [f"custom/Patterns/{command}" for _, command in custom_swarm_commands["Patterns"]]        

        if self.GUI_command.data in valid_control_commands:
            self.stored_command = self.GUI_command.data
        
        if no_drones > 0:
            match self.stored_command:
                # Position Controller #
                case "custom/Patterns/activate_pos_commander":
                    grid_points = generate_grid(no_drones)
                    for i, uri in enumerate(self.controller.get_swarming_uris()):
                        self.controller.set_position(uri, grid_points[i])
                    self.controller.send_commands()
                
                # Velocity Controller #
                case "custom/Patterns/activate_vel_commander":
                    for uri in self.controller.get_swarming_uris():
                        pos = self.controller.get_position(uri)
                        vel = self.controller.get_velocity(uri)
                        self.controller.set_velocity(uri, generate_velocities(pos, vel, set_speed=1.0))
                    self.controller.send_commands()

                # Rotating Diamond #
                case "custom/Patterns/activate_rotating_diamond":
                    grid_points = generate_rotating_diamond()
                    for i, uri in enumerate(self.controller.get_swarming_uris()):
                        if i <= 5: # pattern supports 8 drones
                            self.controller.set_position(uri, grid_points[i])
                        else: # for the remaining drones, have them fly around randomly 
                            pos = self.controller.get_position(uri)
                            vel = self.controller.get_velocity(uri)
                            self.controller.set_velocity(uri, generate_velocities(pos, vel, set_speed=1.0))
                    self.controller.send_commands()

                # Horizontally Rotating Lines #
                case "custom/Patterns/activate_hor_rotating_lines":
                    grid_points = generate_hor_rotating_lines()
                    for i, uri in enumerate(self.controller.get_swarming_uris()):
                        if i <= 7:  # pattern supports 8 drones
                            self.controller.set_position(uri, grid_points[i])
                        else: # for the remaining drones, have them fly around randomly 
                            pos = self.controller.get_position(uri)
                            vel = self.controller.get_velocity(uri)
                            self.controller.set_velocity(uri, generate_velocities(pos, vel, set_speed=1.0))
                    self.controller.send_commands()

                # Vertically Rotating Lines #
                case "custom/Patterns/activate_ver_rotating_lines":
                    grid_points = generate_ver_rotating_lines()
                    for i, uri in enumerate(self.controller.get_swarming_uris()):
                        if i <= 7: # pattern supports 8 drones
                            self.controller.set_position(uri, grid_points[i])
                        else: # for the remaining drones, have them fly around randomly 
                            pos = self.controller.get_position(uri)
                            vel = self.controller.get_velocity(uri)
                            self.controller.set_velocity(uri, generate_velocities(pos, vel, set_speed=1.0))   
                    self.controller.send_commands()

                # Sin Wave #
                case "custom/Patterns/activate_sin_wave":
                    grid_points = generate_sinwave()
                    for i, uri in enumerate(self.controller.get_swarming_uris()):
                        if i <= 7: # pattern supports 8 drones
                            self.controller.set_position(uri, grid_points[i])
                        else: # for the remaining drones, have them fly around randomly 
                            pos = self.controller.get_position(uri)
                            vel = self.controller.get_velocity(uri)
                            self.controller.set_velocity(uri, generate_velocities(pos, vel, set_speed=1.0))   
                    self.controller.send_commands()


def main(args=None):
    rclpy.init(args=args)
    node = MasterCommander()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()