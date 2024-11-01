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

def generate_waypoints(timesteps, amplitude, landing_cycles):
    points = np.array([[1, 1, 0], [1, -1, 0], [-1, -1, 0], [-1, 1, 0]])
    num_points = len(points)
    idx = 1
    # Total segments will be equal to the number of waypoints - 1
    segments = timesteps - 1
    
    # Calculate waypoints
    waypoints = []
    
    for i in range(num_points):
        start = points[i]
        end = points[(i + 1) % num_points]
        
        for t in range(segments // num_points + 1):
            alpha = t / (segments // num_points)
            x = (1 - alpha) * start[0] + alpha * end[0]
            y = (1 - alpha) * start[1] + alpha * end[1]
            z = amplitude*np.sin(np.pi * alpha)  # Sinusoidal Z coordinate
            
            waypoints.append([x, y, z, idx])
            idx += 1

        for _ in range(landing_cycles):
            waypoints.append([-1, -1, -1, idx])
            idx += 1
    
    return np.array(waypoints)  # Make sure we have exactly `timesteps` waypoints



class WaypointPublisher(Node):

    def __init__(self):
        super().__init__('waypoint_publisher')
        self.waypoint_pub_ = self.create_publisher(Polygon, '/waypoints', 10)
        self.pattern_switch_sub_ = self.create_subscription(String, '/ROS_pattern_switch', self.pattern_switch_callback, 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.current_pattern_function = self.generate_grid
        self.command_pub_ = self.create_publisher(ControllerCommand, 'controller_command', 10)

        self.robot_data = read_robot_data('/home/ruov/Lighthouse_Swarm_Controller/ros2_ws/src/waypoint_publisher/waypoint_publisher/buggy circle path_20241031_135452.txt')  # Replace with the actual path
        
        self.waypoints1 = self.robot_data[1]['waypoints'] if 1 in self.robot_data else []
        self.waypoints2 = self.robot_data[2]['waypoints'] if 2 in self.robot_data else []

        self.orientations1 = self.robot_data[1]['orientations'] if 1 in self.robot_data else []
        self.orientations2 = self.robot_data[2]['orientations'] if 2 in self.robot_data else []

        
        self.waypoint_idx = 0
        self.total_waypoints = len(self.waypoints1)

        # self.get_logger().info(self.robot_data)



    def timer_callback(self):
        vertices = self.current_pattern_function()

        if vertices is not None:
            msg = Polygon()
            for vertex in vertices:
                point = Point32()
                self.get_logger().info(f'{vertex}')

                point.x, point.y, point.z = float(vertex[0]), float(vertex[1]), float(vertex[2])
                msg.points.append(point)

            self.waypoint_pub_.publish(msg)

            self.get_logger().info('Publishing:')
            for vertex in vertices:
                self.get_logger().info(f'  [{vertex[0]}, {vertex[1]}, {vertex[2]}]')
        else:
            pass

    def pattern_switch_callback(self, msg):
        command = msg.data
        self.get_logger().info(command)
        if command == "custom/Patterns/activate_pos_commander":
            self.current_pattern_function = self.generate_grid
            self.get_logger().info("generate grid")
        elif command == "custom/Patterns/activate_vel_commander":
            self.get_logger().info("NOT SUPPORTED COMMAND")
        elif command == "custom/Patterns/activate_rotating_diamond":
            self.current_pattern_function = self.generate_rotating_diamond
            self.get_logger().info("generate rotating diamond")
        elif command == "custom/Patterns/activate_hor_rotating_lines":
            self.current_pattern_function = self.generate_hor_rotating_lines
            self.get_logger().info("generate horizontal rotating lines")
        elif command == "custom/Patterns/activate_ver_rotating_lines":
            self.current_pattern_function = self.generate_ver_rotating_lines
            self.get_logger().info("generate vertical rotating lines")
        elif command == "custom/Patterns/activate_sin_wave":
            self.current_pattern_function = self.generate_sinwave
            self.get_logger().info("generate sinwave")
        elif command == "custom/Patterns/activate_landing_test":
            self.current_pattern_function = self.generate_landing_test
            self.get_logger().info("generate landing test")
        elif command == "custom/Patterns/activate_landing_testII":
            self.current_pattern_function = self.generate_landing_testII
            self.get_logger().info("generate landing testII")
        elif command == "custom/Patterns/activate_inspection_dual":
            self.current_pattern_function = self.generate_inspection_dual
            self.get_logger().info("generate inspection dual")
        else:
            self.get_logger().info("NOT IMPLEMENTED")

    def generate_grid(self):
        no_drones = 8
        spacing = 0.5
        height = 1.0
        offset = np.array([0.0, 0.0])

        grid_size = math.ceil(math.sqrt(no_drones))
        grid = []
        for x in range(grid_size):
            for y in range(grid_size):
                grid.append(np.array([
                    (x - (grid_size - 1) / 2.0) * spacing + offset[0],
                    (y - (grid_size - 1) / 2.0) * spacing + offset[1],
                    height
                ]))
        return grid

    def generate_velocities(self, pos, vel, height, turn_scaler, set_speed):
        v_origin = np.array([0.0, 0.0, height]) - pos
        a = v_origin - (np.dot(v_origin, vel) / np.linalg.norm(vel)**2) * vel
        a = a / np.linalg.norm(a) * turn_scaler
        new_vel = vel + a

        if set_speed > 0:
            new_vel = new_vel / np.linalg.norm(new_vel) * set_speed

        return new_vel

    def generate_rotating_diamond(self):
        center = np.array([0.0, 0.0, 1.25])
        max_distance = 0.75
        frequency = 0.1
        time_interval = 1.0 / frequency

        vertices = [
            center + np.array([0, 0, max_distance]),
            center - np.array([0, 0, max_distance]),
            center - np.array([max_distance, 0, 0]),
            center + np.array([max_distance, 0, 0]),
            center + np.array([0, max_distance, 0]),
            center - np.array([0, max_distance, 0])
        ]

        t = time.time()
        angle = 2 * math.pi * (t % time_interval) / time_interval

        rotation_matrix = np.array([
            [math.cos(angle), -math.sin(angle), 0],
            [math.sin(angle), math.cos(angle), 0],
            [0, 0, 1]
        ])

        rotated_vertices = [np.dot(rotation_matrix, vertex) for vertex in vertices]

        return rotated_vertices

    def generate_hor_rotating_lines(self):
        pass


    def generate_ver_rotating_lines(self):
        R = 0.8
        d = 0.7
        z0 = 0.3

        frequency = 0.05
        time_interval = 1.0 / frequency

        vertices = [
            np.array([R, 0, z0 + 0 * d]),
            np.array([R, 0.1, z0 + 1 * d]),
            np.array([R, -0.1, z0 + 2 * d]),
            np.array([R, 0, z0 + 3 * d]),
            np.array([-R, 0, z0 + 0 * d]),
            np.array([-R, 0.1, z0 + 1 * d]),
            np.array([-R, -0.1, z0 + 2 * d]),
            np.array([-R, 0, z0 + 3 * d])
        ]

        t = time.time()
        angle = 2 * math.pi * (t % time_interval) / time_interval

        rotation_matrix = np.array([
            [math.cos(angle), -math.sin(angle), 0],
            [math.sin(angle), math.cos(angle), 0],
            [0, 0, 1]
        ])

        vertices = [np.dot(rotation_matrix, vertex) for vertex in vertices]

        return vertices

    def generate_spiral(self):
        R = 0.5
        z0 = 0.25

        frequency = 0.2
        time_interval = 1.0 / frequency
        
        dth = 60.0 / 57.3
        dh = 0.3

        vertices = [
            np.array([R * math.cos(0 * dth), R * math.sin(0 * dth), 0 * dh + z0]),
            np.array([R * math.cos(1 * dth), R * math.sin(1 * dth), 1 * dh + z0]),
            np.array([R * math.cos(2 * dth), R * math.sin(2 * dth), 2 * dh + z0]),
            np.array([R * math.cos(3 * dth), R * math.sin(3 * dth), 3 * dh + z0]),
            np.array([R * math.cos(4 * dth), R * math.sin(4 * dth), 4 * dh + z0]),
            np.array([R * math.cos(5 * dth), R * math.sin(5 * dth), 5 * dh + z0]),
            np.array([R * math.cos(6 * dth), R * math.sin(6 * dth), 6 * dh + z0]),
            np.array([R * math.cos(7 * dth), R * math.sin(7 * dth), 7 * dh + z0]),
            np.array([R * math.cos(8 * dth), R * math.sin(8 * dth), 9 * dh + z0])
        ]

        t = time.time()
        angle = 2 * math.pi * (t % time_interval) / time_interval

        rotation_matrix = np.array([
            [math.cos(angle), -math.sin(angle), 0],
            [math.sin(angle), math.cos(angle), 0],
            [0, 0, 1]
        ])

        vertices = [np.dot(rotation_matrix, vertex) for vertex in vertices]

        return vertices

    def generate_smiley(self):
        frequency = 0.05
        time_interval = 1.0 / frequency

        vertices = [
            np.array([0, 0.5, 1.5]),
            np.array([0, -0.5, 1.5]),
            np.array([0, -0.8, 1.0]),
            np.array([0, -0.5, 0.6]),
            np.array([0, -0.2, 0.4]),
            np.array([0, 0.2, 0.4]),
            np.array([0, 0.5, 0.6]),
            np.array([0, 0.8, 1.0])
        ]

        t = time.time()
        angle = 2 * math.pi * (t % time_interval) / time_interval

        rotation_matrix = np.array([
            [math.cos(angle), -math.sin(angle), 0],
            [math.sin(angle), math.cos(angle), 0],
            [0, 0, 1]
        ])

        vertices = [np.dot(rotation_matrix, vertex) for vertex in vertices]

        return vertices

    def generate_sinwave(self):
        x_min = -1.0
        x_max = 1.0
        num_drones_per_line = 4
        num_lines = 2

        x_coordinates = np.linspace(x_min, x_max, num_drones_per_line)

        grid_points = []

        amplitude = 0.4
        frequency = 0.8

        t = time.time()

        for line_index in range(num_lines):
            y = (line_index - (num_lines - 1) / 2.0) * 0.75
            z_coordinates = 1.0 + amplitude * np.sin(frequency * t - x_coordinates)

            for i in range(num_drones_per_line):
                grid_points.append(np.array([x_coordinates[i], y, z_coordinates[i]]))

        return grid_points

    def generate_landing_test(self):
        if self.waypoint_idx < self.total_waypoints:
            waypoint = self.waypoints1[self.waypoint_idx]
            self.waypoint_idx += 1

            self.get_logger().info("\n\n\WAYPOINT: ")
            self.get_logger().info(f'{waypoint}')

            # vertices = list(waypoints)
            # for vertex in vertices:
            #     self.get_logger().info(f'  [{waypoint[0]}, {waypoint[1]}, {waypoint[2]}]')

            return list([waypoint])
        else:
            return None  # Signal to stop or reset

        return vertices

    def generate_landing_testII(self):
        if self.waypoint_idx < self.total_waypoints:
            waypoint = self.waypoints1[self.waypoint_idx]
            self.waypoint_idx += 1

            self.get_logger().info("\n\n\WAYPOINT: ")
            self.get_logger().info(f'{waypoint}')

            # vertices = list(waypoints)
            # for vertex in vertices:
            #     self.get_logger().info(f'  [{waypoint[0]}, {waypoint[1]}, {waypoint[2]}]')

            return list([waypoint])
        else:
            return None  # Signal to stop or reset

        return vertices

    def generate_inspection_dual(self):
        if self.waypoint_idx < self.total_waypoints:
            waypoint1 = self.waypoints1[self.waypoint_idx]
            waypoint2 = self.waypoints2[self.waypoint_idx]
            self.waypoint_idx += 1

            self.get_logger().info("\n\n\WAYPOINT: ")
            self.get_logger().info(f'{waypoint1}')
            self.get_logger().info(f'{waypoint2}')
            # vertices = list(waypoints)
            # for vertex in vertices:
            #     self.get_logger().info(f'  [{waypoint[0]}, {waypoint[1]}, {waypoint[2]}]')

            return list([waypoint1, waypoint2])
        else:
            return None  # Signal to stop or reset

        return vertices
    
    



def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
