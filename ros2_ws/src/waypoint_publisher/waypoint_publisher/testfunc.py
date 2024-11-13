import math
import time
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Polygon, Point32
from topic_interface.msg import ControllerCommand




class WaypointPublisher(Node):

    def __init__(self):
        super().__init__('waypoint_publisher')

        # Publishers and subscribers
        self.waypoint_pub_ = self.create_publisher(Polygon, '/waypoints', 10)
        self.command_pub_ = self.create_publisher(ControllerCommand, 'controller_command', 10)
        self.pattern_switch_sub_ = self.create_subscription(String, '/ROS_pattern_switch', self.pattern_switch_callback, 10)
        
        # Timer for publishing waypoints
        self.timer_ = self.create_timer(1.0, self.timer_callback)

        # Load waypoints from file
        self.robot_data = read_robot_data('path/to/your/waypoints.txt')  # Replace with the actual path
        self.robot_id = 1  # Specify the robot ID you want to control
        self.waypoints = self.robot_data[self.robot_id]['waypoints'] if self.robot_id in self.robot_data else []
        self.orientations = self.robot_data[self.robot_id]['orientations'] if self.robot_id in self.robot_data else []
        
        self.waypoint_idx = 0
        self.total_waypoints = len(self.waypoints)

        # Set initial pattern function
        self.current_pattern_function = self.get_next_waypoint

    def timer_callback(self):
        vertices = self.current_pattern_function()
        if vertices is not None:
            msg = Polygon()
            point = Point32()
            point.x, point.y, point.z = float(vertices[0]), float(vertices[1]), float(vertices[2])
            msg.points.append(point)
            self.waypoint_pub_.publish(msg)

            # Log the current waypoint
            self.get_logger().info(f'Publishing waypoint: [{vertices[0]}, {vertices[1]}, {vertices[2]}]')
        else:
            # End of waypoints - trigger landing if desired
            self.get_logger().info("No more waypoints, initiating landing")
            msg = ControllerCommand()
            msg.uri = "all"
            msg.data = "land in place"
            self.command_pub_.publish(msg)

    def get_next_waypoint(self):
        if self.waypoint_idx < self.total_waypoints:
            waypoint = self.waypoints[self.waypoint_idx]
            self.waypoint_idx += 1
            return waypoint
        else:
            return None  # Signal to stop or reset

    def pattern_switch_callback(self, msg):
        command = msg.data
        if command == "activate_path_from_file":
            self.current_pattern_function = self.get_next_waypoint
            self.get_logger().info("Switched to path from file")
        else:
            self.get_logger().info("Pattern command not supported")

    # Define additional pattern functions here if needed, similar to the original node

class WaypointPublisher(Node):

    def __init__(self):
        super().__init__('waypoint_publisher')
        self.waypoint_pub_ = self.create_publisher(Polygon, '/waypoints', 10)
        self.pattern_switch_sub_ = self.create_subscription(String, '/ROS_pattern_switch', self.pattern_switch_callback, 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.current_pattern_function = self.generate_grid
        self.command_pub_ = self.create_publisher(ControllerCommand, 'controller_command', 10)

        # Load waypoints from file (assuming read_robot_data returns a dictionary of waypoints)
        self.waypoints = generate_waypoints(100, 1.0, 5)
        self.waypoint_len = len(self.waypoints)
        self.waypoint_idx = 0

    def timer_callback(self):
        vertices = self.current_pattern_function()

        # Ensure vertices is a list of [x, y, z] points before processing
        if vertices is not None and isinstance(vertices, list):
            msg = Polygon()
            for vertex in vertices:
                if isinstance(vertex, (list, tuple)) and len(vertex) == 3:
                    point = Point32()
                    point.x, point.y, point.z = float(vertex[0]), float(vertex[1]), float(vertex[2])
                    msg.points.append(point)

            self.waypoint_pub_.publish(msg)

            self.get_logger().info('Publishing:')
            for vertex in vertices:
                self.get_logger().info(f'  [{vertex[0]}, {vertex[1]}, {vertex[2]}]')
        else:
            self.get_logger().info("Invalid vertices format received.")

    def generate_landing_testII(self):
        # Check for waypoint data format before publishing
        idx = self.waypoint_idx
        self.waypoint_idx = (self.waypoint_idx + 1) % self.waypoint_len
        
        waypoint = self.waypoints[idx]
        
        if len(waypoint) >= 3 and waypoint[0] >= 0:
            return [waypoint[0], waypoint[1], waypoint[2]]
        
        # Land if conditions indicate landing should occur
        msg = ControllerCommand()
        msg.uri = "all"
        msg.data = "land in place"
        self.command_pub_.publish(msg)
        self.get_logger().info('\n\n Landing \n')
        
        return None