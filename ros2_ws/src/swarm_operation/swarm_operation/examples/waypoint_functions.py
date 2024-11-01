import math
import time
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Polygon, Point32
from topic_interface.msg import ControllerCommand

# Assuming `read_robot_data` is defined as before and imported here
def read_robot_data(file_path):
    """
    Reads the robot data from a txt file and returns it as a dictionary.
    """
    robot_data = {}
    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if line:
                parts = line.split()
                robot_num = int(parts[0])
                x, y, z = map(float, parts[1:4])
                pitch, yaw = map(float, parts[4:6])
                if robot_num not in robot_data:
                    robot_data[robot_num] = {"waypoints": [], "orientations": []}
                robot_data[robot_num]["waypoints"].append((x, y, z))
                robot_data[robot_num]["orientations"].append((pitch, yaw))
    return robot_data


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
        self.robot_data = read_robot_data('ros2_ws/src/swarm_operation/swarm_operation/examples/buggy circle path_20241031_135452.txt')  # Replace with the actual path
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


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()