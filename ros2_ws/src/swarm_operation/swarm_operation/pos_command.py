#!/usr/bin/env python3

"""
Node that publishes a velocity command to a drone
2023 - Emergent Swarm Solutions (ESS)
"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import String
from topic_interface.msg import StringList
import numpy as np
import time
import random

class PosCommandDemo(Node):
    
    def __init__(self):
        super().__init__("position_commander",
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)
        
        # subscriptions dict that listens to the state of each drone
        self.drone_subs = {}
        self.drone_uris = []
        self.drone_states = {}
        self.drone_positions = {}

        self.num_radios = self.get_parameter('number_radios').value

        # see how many drones are swarming
        self.no_swarming = 0

        self.time_running = time.time()

        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


        # create a subscriber that listens to radios for the uris of the drones
        self.drone_uris_subs = []
        self.drone_parameters_subs = []
        for i in range(self.num_radios):
            self.drone_uris_subs.append(self.create_subscription(StringList, f'ID{i}/self_uris', lambda msg, radio=i: self.update_drone_uris(msg, radio), qos_profile=latching_qos))
            self.drone_parameters_subs.append(self.create_subscription(StringList, f'ID{i}/drone_parameters', lambda msg, radio=i: self.update_drone_parameters(msg, radio), 10))
            self.drone_uris.append(dict())

        # subscribers
        self.position_commander = self.create_publisher(String, 'position_command', 10)
        #self.send_position_timer = self.create_timer(15, self.publish_commands)
        self.GUI_command_sub = self.create_subscription(String, 'GUI_command', self.GUI_command_callback, 10)

        # pattern state machine
        self.state = "random"
        self.prev_state = 'random'
        self.state_func = {
            "random": self.random,
            "circle": self.circle,
            "ellipse_circle": self.ellipse_circle,
            "grid": self.grid,
            "error": self.error
        }

        # list of position commander states
        self.states = ['random', 'circle', 'ellipse_circle', 'grid']
        self.state_index = 0

        # Create a callback for self.loop() which belongs to a personal MutuallyExclusiveCallbackGroup such that other callbacks can be executed in parallel
        loop_group = MutuallyExclusiveCallbackGroup()

        # update rate for each pattern ['random', 'circle', 'ellipse_circle', 'grid']
        self.update_rate = [5, 15, 0.3, 7]
        self.time_per_pattern = [0,0,0,10000]
        self.time_last_sent = time.time()
        self.time_change_pattern = time.time()

        # track previous state of patterns
        self.circle_index = 0
        self.ellipse_index = 0

        # grid
        self.grid_pattern = 0

        self.state_start = True
        
        self.comms = ''

        # update patterns if number of drones change
        self.update_pattern_ = self.create_timer(0.5, self.update_pattern)

        # create state machine loop
        self.loop_call = self.create_timer(0.1, self.callback_loop, callback_group=loop_group)
    

    def update_pattern(self):
        # see how many drones are swarming
        self.no_swarming = 0
        for uri in self.drone_states:
            if self.drone_states[uri] == 'swarming':
                self.no_swarming += 1

        # update swarming patterns
        if self.state == 'circle':
            self.sequence = self.gen_circle()
        if self.state == 'ellipse_circle':
            self.seq_ellipse, self.seq_circle0, self.seq_circle1 = self.gen_ellipse_circle()
        if self.state == 'grid':
            self.sequence = self.gen_grid()

        # change the pattern every x seconds
        if time.time() - self.time_change_pattern > self.time_per_pattern[self.state_index] or self.time_per_pattern[self.state_index] == 0:
            self.state_index = (self.state_index + 1) % len(self.states)
            self.state = self.states[self.state_index]
            self.time_change_pattern = time.time()

    def update_drone_states(self, msg, uri):
        # crazyflie state
        self.drone_states[uri] = msg.data 


    def update_drone_parameters(self, msg, radio):
        if len(self.drone_uris[radio]) > 0:
            for i, param in enumerate(msg.sl):
                # drone parameters: x/y/z//vx/vy/vz/yaw//battery state/battery voltage/lighthouse active/sys_isFlying/sys_isTumbled
                posvel, system_state = [x.split("/") for x in param.split("//")]
                uri = self.drone_uris[radio][i]

                self.drone_positions[uri] = [float(posvel[0]), float(posvel[1]), float(posvel[2])]

    def update_drone_uris(self, msg, radio):
        # if a drone uri is not yet there, add the uri to the dict
        for i, uri in enumerate(msg.sl):
            self.drone_uris[radio][i] = uri

            if uri not in self.drone_subs:
                self.drone_subs[uri] = self.create_subscription(String, 'E' + uri.split('/')[-1] + '/state', lambda msg, uri_i=uri: self.update_drone_states(msg, uri_i), 10)
                self.drone_states[uri] = 'initialising'


    ############################# State machine #############################

    def random(self):
        # make pattern at state start
        if self.state_start:
            self.sequence = self.gen_random_pattern()

        if time.time() - self.time_last_sent > self.update_rate[0] or self.state_start:
            # make empty message
            self.comms = ''

            if self.no_swarming != 0:
                # pick a random item from the sequence
                for uri in self.drone_states:
                    comm = self.sequence[random.randint(0, len(self.sequence)-1)]
                    self.comms += uri.split('/')[-1] + '/' + str(comm[0]) + '/' + str(comm[1]) + '/' + str(comm[2]) + '/' + str(0) + '/'

                self.publish_commands()
                self.time_last_sent = time.time()
                

    def circle(self):
        # make pattern at state start
        if self.state_start:
            self.sequence = self.gen_circle()

        if time.time() - self.time_last_sent > self.update_rate[1] or self.state_start:
            # make empty message
            self.comms = ''
            
            if self.no_swarming != 0:
                # check if circle index is 0 or 1 (one or other side)
                if self.circle_index == 0:
                    self.circle_index = 1
                    offset = int(self.no_swarming/2)
                else:
                    self.circle_index = 0
                    offset = 0
            
                # pick a random item from the sequence
                index = 0
                for uri in self.drone_states:
                    if self.drone_states[uri] == 'swarming':
                        comm = self.sequence[(offset + index) % self.no_swarming]
                        self.comms += uri.split('/')[-1] + '/' + str(comm[0]) + '/' + str(comm[1]) + '/' + str(comm[2]) + '/' + str(0) + '/'
                        index += 1

                # publish the command
                self.publish_commands()
                self.time_last_sent = time.time()

    def ellipse_circle(self):
        # make pattern at state start
        if self.state_start:
            self.seq_ellipse, self.seq_circle0, self.seq_circle1 = self.gen_ellipse_circle()
    

        if self.no_swarming != 0:
            if time.time() - self.time_last_sent > self.update_rate[2] or self.state_start:
                # make empty message
                self.comms = ''

                self.ellipse_index = (self.ellipse_index + 1) % len(self.seq_ellipse)
                circle_index = (self.ellipse_index) % len(self.seq_circle0)

                index = 0
                for uri in self.drone_states:
                    if self.drone_states[uri] == 'swarming':
                        if index == 0:
                            comm = self.seq_circle0[circle_index]
                        elif index == 1:
                            comm = self.seq_circle1[circle_index]
                        elif index > 1:
                            new_index = int((self.ellipse_index + (index-2)* len(self.seq_ellipse) / (self.no_swarming - 1)) % len(self.seq_ellipse))
                            comm = self.seq_ellipse[new_index]
                        self.comms += uri.split('/')[-1] + '/' + str(comm[0]) + '/' + str(comm[1]) + '/' + str(comm[2]) + '/' + str(0) + '/'
                        index += 1
                
                self.publish_commands()
                self.time_last_sent = time.time()

    def grid(self):
        # make pattern at state start
        if self.state_start:
            self.sequence = self.gen_grid()

        if time.time() - self.time_last_sent > self.update_rate[3] or self.state_start:
            # make empty message
            self.comms = ''
            if self.no_swarming != 0:
                # pick a random item from the sequence
                if self.grid_pattern == 0:
                    self.grid_pattern = 1
                    index = 0
                    increment = 1
                else:
                    self.grid_pattern = 0
                    index = len(self.sequence) - 1
                    increment = -1

                for uri in self.drone_states:
                    if self.drone_states[uri] == 'swarming':
                        comm = self.sequence[index]
                        self.comms += uri.split('/')[-1] + '/' + str(comm[0]) + '/' + str(comm[1]) + '/' + str(comm[2]) + '/' + str(0) + '/'
                        index += increment

                self.publish_commands()
                self.time_last_sent = time.time()


    def difference(self, vec1 ,vec2):
        return [vec1[0]-vec2[0], vec1[1]-vec2[1], vec1[2]-vec2[2]]

    def mag_dif(self, vec1, vec2):
        return np.linalg.norm(self.difference(vec1, vec2))
    
    def add_vector(self, vec1, vec2):
        return [vec1[0]+vec2[0], vec1[1]+vec2[1], vec1[2]+vec2[2]]

    def publish_commands(self):
        # publish the commands
        msg = String()
        msg.data = self.comms
        self.position_commander.publish(msg)
    
    def error(self):
        pass

    ############################# Generate patterns #############################

    def gen_random_pattern(self):
        # make the first pattern
        number_steps = 4
        stepx, stepy, stepz = 0.375, 0.375, 0.1
        x0 = -0.75 + 0.5
        y0 = -0.75 + 0.5
        z0 = 1.0
        sequence = []

        for i in range(number_steps):
            x = x0 + i * stepx
            for j in range(number_steps):
                y = y0 + j * stepy
                for k in range(number_steps):
                    z = z0 + k * stepz
                sequence.append([x,y,z])
        return sequence
    
    def gen_grid(self):
        # make the grid pattern
        sequence = []
        if self.no_swarming == 0:
            steps = 0
        elif self.no_swarming == 1:
            steps = 1
            separation = 0
            x0, y0, z0 = -0.5, 0.0, 1.0
        elif self.no_swarming < 4:
            steps = 2
            separation = 0.4
            x0, y0, z0 = -0.5, 0.0, 1.0
        elif self.no_swarming < 9:
            steps = 3
            separation = 0.4
            x0, y0, z0 = -0.5, 0.0, 1.0
        elif self.no_swarming < 16:
            steps = 4
            separation = 0.4
            x0, y0, z0 = -0.5, 0.0, 1.0
        elif self.no_swarming < 25:
            steps = 5
            separation = 0.35
            x0, y0, z0 = -0.5, 0.0, 1.0
        elif self.no_swarming < 36:
            steps = 6
            separation = 0.35
            x0, y0, z0 = -0.9, -0.4, 1.0
        
        if self.no_swarming != 0 and steps != 0:
            # make the grid pattern for each step
            for i in range(steps):
                x = x0 + i * separation
                for j in range(steps):
                    y = y0 + j * separation
                    sequence.append([x,y,z0])
        
        return sequence


    def gen_circle(self):
        sequence = []
        if self.no_swarming != 0:
            # number of points
            dphi = 2*np.pi / self.no_swarming

            # make a circle of points
            for i in range(self.no_swarming):
                phi = i * dphi
                x = 0.8 * np.cos(phi) + 0.0
                y = 0.8 * np.sin(phi) + 0.0
                z = 0.8
                sequence.append([x,y,z])
        return sequence

    def gen_ellipse_circle(self):
        # make a pattern where drones fly in an ellipse and other drones fly through it
        CE_ellipse = []
        CE_circle1 = []
        CE_circle2 = []
        no_points_ellipse = 50
        no_points_circle = 20

        # make the ellipse
        a,b = 1, 1.5
        c = 0.75
        x = np.linspace(-0.5,1.0,int(no_points_ellipse/2))
        z = ((c**2 - (a*x-0.25)**2) / b)**0.5

        # make the ellipse list
        for i in range(len(x)):
            CE_ellipse.append([0.5,x[i]-(z[i]+1.0)*0.4,z[i]+1.0])
            CE_ellipse.insert(0,[0.5,x[i]+(z[i]+1.0)*0.4,-z[i]+1.0])

        # make the circles
        dphi = 2*np.pi / no_points_circle
        for i in range(no_points_circle):
            phi = i * dphi
            x = 0.9 * np.cos(phi) + 0.5
            y = 0.9 * np.sin(phi) + 1.0
            z = 1.0
            CE_circle1.append([x,y,z])

        for i in range(no_points_circle):
            phi = i * dphi
            x = 0.9 * np.cos(phi) + 0.5
            y = 0.9 * np.sin(phi) + -0.5
            z = 1.0
            CE_circle2.append([x,y,z])

        return CE_ellipse, CE_circle1, CE_circle2

    def callback_loop(self):
        if self.prev_state != self.state:
            self.get_logger().info(self.state)
            self.prev_state = self.state
            self.state_start = True

        try:
            self.state_func[self.state]()
            self.state_start = False
        except Exception as e:
            self.get_logger().info(f"error in {self.state}: {e}")
            self.state = "error"
    

    def GUI_command_callback(self, msg):
        if msg.data == "terminate/kill all":
            self.destroy_node()
            sys.exit()

def main(args=None):
    rclpy.init(args=args)
    node = PosCommandDemo()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()