import random
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ..helper_classes import SwarmController
from .waypoint_functions import *
from ..config import LH_HIGH_RISK_BOUNDS, ABS_BOUNDS

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
        ("Leader-Follower", "activate_leader_follower")
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

        # start timers
        self.main_loop_timer = self.create_timer(0.5, self.main_loop_cb)
        self.leader_timer = self.create_timer(15, self.leader_cb)

        # Init leader variables
        self.leader_uris = []
        self.FORMATION_OFFSETS = [
            [0.15, 0.15, 0.0],
            [-0.15, 0.15, 0.0],
            [0.15, -0.15, 0.0],
            [-0.15, -0.15, 0.0]
        ]  # Example formation offsets
        
    
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

                # Leader-follower Behaviour #
                case "custom/Patterns/activate_leader_follower":
                    flattened_bounds = [bound for axis_bounds in LH_HIGH_RISK_BOUNDS for bound in axis_bounds]
                    drone_uris = self.controller.get_swarming_uris()
                    drone_positions = {uri: self.controller.get_position(uri) for uri in drone_uris}

                    for index, uri in enumerate(drone_uris):
                        pos = drone_positions[uri]
                        all_positions = list(drone_positions.values())
                        if not self.leader_uris:  # If no leaders are set, give random velocities
                            self.controller.set_velocity(uri, generate_repelled_velocities_in_cage(pos, all_positions, set_speed=0.5, bounds=flattened_bounds))
                        else:
                            if uri in self.leader_uris:
                                # Only leaders get new velocities
                                self.controller.set_velocity(uri, generate_repelled_velocities_in_cage(pos, all_positions, set_speed=0.5, bounds=flattened_bounds))
                            else:
                                # Assign the drone to the nearest leader with some randomness
                                nearest_leader = self.assign_to_leader(uri, drone_positions)
                                # Set the position to maintain the formation
                                self.set_formation_position(uri, nearest_leader, drone_positions, index)

                    self.controller.send_commands()
   
    def leader_cb(self):
        """Leader callback function, changes the leaders of the swarm"""
        uris = self.controller.get_swarming_uris()
        num_drones = len(uris)
        if num_drones:
            self.get_logger().info("Changing leaders")
            # Determine the number of leaders (2 or 3) based on the number of drones
            num_leaders = 2 if num_drones >= 3 else 1
            # Randomly select new leaders
            new_leaders = random.sample(uris, min(num_drones, num_leaders))
            self.leader_uris = new_leaders
            self.get_logger().info(f"New leaders: {self.leader_uris}")

    def assign_to_leader(self, uri, drone_positions):
        """Assign the drone to the nearest leader with some randomness"""
        distances = {leader: self.calculate_distance(drone_positions[uri], drone_positions[leader]) for leader in self.leader_uris}
        sorted_leaders = sorted(distances, key=distances.get)

        # Introduce randomness: 10% chance to pick a random leader instead of the closest
        if random.random() < 0.1:
            return random.choice(sorted_leaders)
        else:
            return sorted_leaders[0]
        
    def set_formation_position(self, uri, leader_uri, drone_positions, index):
        """Set the position of the drone to maintain a formation relative to the leader"""
        leader_pos = drone_positions[leader_uri]
        assigned_offset = self.FORMATION_OFFSETS[index % len(self.FORMATION_OFFSETS)]  # Assign offset based on index

        # Adjust the position based on the formation offset
        formation_pos = [
            leader_pos[0] + assigned_offset[0],
            leader_pos[1] + assigned_offset[1],
            leader_pos[2] + assigned_offset[2]
        ]
        self.controller.set_position(uri, formation_pos)

    def calculate_distance(self, pos1, pos2):
        """Calculate the Euclidean distance between two positions"""
        return ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2 + (pos1[2] - pos2[2]) ** 2) ** 0.5


def main(args=None):
    rclpy.init(args=args)
    node = MasterCommander()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()