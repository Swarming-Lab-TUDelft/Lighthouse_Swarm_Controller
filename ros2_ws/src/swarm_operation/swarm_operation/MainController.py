#!/usr/bin/env python3

"""
Node that controls which drones should take off and land.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from std_msgs.msg import String, UInt16
from topic_interface.msg import ControllerCommand, StringList

import sys

class ControllerMain(Node):

    def __init__(self):
        super().__init__("control_main",
                        # allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)
        
        # subscriptions dict that listens to the state of each drone
        self.drone_subs = {}

        # radio dict with all uri's grouped per radio
        self.num_radios = self.get_parameter('number_radios').value
        self.radios_uri = {}
        self.radio_state_subs = []
        self.radio_states = {}
        self.radios_ready = False
        for i in range(self.num_radios):
            self.radios_uri[str(i)] = []
            self.radio_state_subs.append(self.create_subscription(String, f'ID{i}/radio_state', lambda msg, radio=i: self.update_radio_states(msg, radio), 10))
            self.radio_states[i] = 'not found'
        self.reconnecting = False
        self.reconnecting_radio = 0
        self.reconnect_request_sent = False

        # create latching qos profile
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # initial swarm state
        self.req_nr_swarming = 0
        self.no_swarming = 0

        # number of running pad requests
        self.landing_cfs = 0

        # dictionary of Crazyflies with states
        self.requested_pad = {}
        self.request_take_off = {}
        self.drone_states = {}
        
        # states of drones relevant for the main controller 
        self.mc_states = {}

        self.GUI_commands = {
            "add drone": lambda msg_i="add": self.update_req_cfs(msg_i),
            "remove drone": lambda msg_i="remove": self.update_req_cfs(msg_i),
            "return all": self.return_all,
            "emergency land": self.emergency_land,
            "terminate": self.terminate_callback,
            "land in place one": self.land_one,
            "return one": self.return_one,
        }

        # subscriptions
        # self.update_req_cfs_sub = self.create_subscription(String, 'req_cfs', self.update_req_cfs, 10)
        self.GUI_command_sub = self.create_subscription(String, 'GUI_command', self.GUI_command_callback, 10)
        # self.return_all_sub = self.create_subscription(String, 'return_all', self.return_all, 10)
        self.request_pad_sub = self.create_subscription(String, "cf_charge_req", self.drone_requested_pad, 10)
        # self.emergency_land_sub = self.create_subscription(String, 'emergency_land', self.emergency_land_callback, 10)

        # publishers
        self.command_pub = self.create_publisher(ControllerCommand, 'controller_command', 10)
        self.reconnect_pub = self.create_publisher(String, 'reconnect', 10)
        self.announcement_pub = self.create_publisher(String, 'controller_announcement', 10)
        self.no_swarming_pub = self.create_publisher(UInt16, 'no_swarming', 10)

        # create a subscriber that listens to radios for the uris of the drones
        self.drone_uris_subs = []
        for i in range(self.num_radios):
            self.drone_uris_subs.append(self.create_subscription(StringList, f'ID{i}/self_uris', lambda msg, radio=i: self.update_drone_uris(msg), qos_profile=latching_qos))

        self.add_drone_to_swarm = self.create_timer(0.5, self.add_drone_too_few_cfs)
        self.swarming_timer = self.create_timer(0.1, self.swarming_callback)

    def update_drone_uris(self, msg):
        # if a drone uri is not yet there, add the uri to the dict
        for i, uri in enumerate(msg.sl):
            if uri not in self.drone_subs:
                self.drone_subs[uri] = self.create_subscription(String, 'E' + uri.split('/')[-1] + '/state', lambda msg, uri_i=uri: self.update_drone_states(msg, uri_i), 10)
                self.drone_states[uri] = 'initialising'
                self.mc_states[uri] = 'initialising'
                self.requested_pad[uri] = False
                self.request_take_off[uri] = False

                # assign drone to radio instance in dict
                self.radios_uri[uri[8]].append(uri)

                # print the radio and connected drones
                # self.get_logger().info(f'Added drone {uri} to radio {uri[8]}')
                # self.get_logger().info(f'Current radios: {self.radios_uri}')

    
    def update_drone_states(self, msg, uri):
        # crazyflie state
        self.drone_states[uri] = msg.data
    
    def update_radio_states(self, msg, radio):
        # radio state
        self.radio_states[radio] = msg.data
        if not self.radios_ready:
            radios_found = True
            radios_ready = True
            for state in self.radio_states.values():
                if state != 'ready':
                    radios_ready = False
                if state != 'radio found':
                    radios_found = False
            
            if radios_ready:
                self.radios_ready = True
                self.announcement_pub.publish(String(data='radios ready'))
            elif radios_found:
                self.announcement_pub.publish(String(data='radios found'))
    
    def return_one(self, uri):
        if self.req_nr_swarming > 0:
            self.req_nr_swarming -= 1
        self.command_pub.publish(ControllerCommand(uri=uri, data='return'))

    def return_all(self):
        # set the required number of drones to zero
        self.req_nr_swarming = 0
        self.command_pub.publish(ControllerCommand(uri='all', data='return'))
        
        # for key in self.drone_states:
        #     if (self.drone_states[key] in ('swarming', 'taking off', 'error')) and not self.requested_pad[key]:
        #         # send request pad to all drones that are swarming
        #         msg = ControllerCommand()
        #         msg.uri = key
        #         msg.data = 'return'
        #         self.command_pub.publish(msg)
        #         # set the drone pad request variable to true
        #         self.requested_pad[key] = True
    
    def land_one(self, uri):
        if self.req_nr_swarming > 0:
            self.req_nr_swarming -= 1
        self.command_pub.publish(ControllerCommand(uri=uri, data='land in place'))

    def emergency_land(self):
        self.req_nr_swarming = 0
        self.command_pub.publish(ControllerCommand(uri='all', data='land in place'))

    def swarming_callback(self):
        # change the mc states
        for key in self.drone_states:
            # go from initialising/waiting to swarming if take off is requested
            if (self.mc_states[key] == 'initialising' or self.mc_states[key] == 'waiting') and self.request_take_off[key]:
                self.request_take_off[key] = False
                self.mc_states[key] = 'swarming'
            if self.drone_states[key] == 'swarming' and not self.requested_pad[key]:
                self.request_take_off[key] = False
                self.mc_states[key] = 'swarming'
            # go from swarming to landing if pad is requested
            if self.mc_states[key] == 'swarming' and self.requested_pad[key]:
                self.mc_states[key] = 'landing'
            # go from landing to waiting if the drone is charging or waiting
            if self.mc_states[key] == 'landing' and (self.drone_states[key] == 'charging' or self.drone_states[key] == 'waiting'):
                self.mc_states[key] = 'charging'
            # go from charging to waiting if the drone is waiting
            if self.mc_states[key] == 'charging' and self.drone_states[key] == 'waiting':
                self.mc_states[key] = 'waiting'
            # go to error if error
            if self.drone_states[key] in ('error', 'error handling'):
                self.mc_states[key] = 'error'
            if self.drone_states[key] not in ('error', 'error handling') and self.mc_states[key] == 'error':
                self.mc_states[key] = 'waiting'
            if self.drone_states[key] == 'disconnected':
                self.mc_states[key] = 'disconnected'
            if self.mc_states[key] == 'disconnected' and self.drone_states[key] != 'disconnected':
                self.mc_states[key] = 'waiting'

        
        # log the composition of the swarm
        self.no_swarming = 0
        self.available_cfs = 0
        self.landing_cfs = 0

        for key in self.mc_states:
            if self.mc_states[key] == 'swarming':
                self.no_swarming += 1
            if (self.mc_states[key] == 'waiting' or self.mc_states[key] == 'initialising'):
                self.available_cfs += 1
            if self.mc_states[key] == 'landing':
                self.landing_cfs += 1
        
        self.no_swarming_pub.publish(UInt16(data=self.req_nr_swarming))

    def update_req_cfs(self, msg):
        # update the required number of drones (from GUI node)
        if msg == 'remove' and self.req_nr_swarming > 0:
            self.req_nr_swarming -= 1
            self.get_logger().info('remove drone from swarm')
        if msg == 'add' and self.req_nr_swarming < self.no_swarming + self.available_cfs:
            self.req_nr_swarming += 1
            self.get_logger().info('add drone to swarm')
    
    def GUI_command_callback(self, msg):
        command = msg.data.split("/")
        if command[0] in self.GUI_commands:
            if len(command) > 1:
                self.GUI_commands[command[0]](*command[1:])
            else:
                self.GUI_commands[command[0]]()

    def add_drone_too_few_cfs(self):
        # set the required number swarming
        if self.req_nr_swarming > self.no_swarming + self.available_cfs and self.req_nr_swarming - self.no_swarming - self.available_cfs != 0:
            self.req_nr_swarming = self.no_swarming + self.available_cfs


        # check if drones need to be added to the swarm
        add_drones = self.req_nr_swarming - self.no_swarming
        if add_drones > 0:
            self.request_charged_drone() 
        
        # check if drones need to be removed from the swarm
        if self.req_nr_swarming < self.no_swarming:
            remove_drones = self.no_swarming - self.req_nr_swarming - self.landing_cfs
            if remove_drones > 0:
                # self.get_logger().info('requesting pad for drone')
                self.request_charging_pad()

    #################### Request a charging pad functions ####################

    def request_charging_pad(self):
        landing_uri = None
        # get the URI of a swarming drone
        for key in self.drone_states:
            if self.drone_states[key] == 'swarming' and not self.requested_pad[key]:
                landing_uri = key
                break
        
        if landing_uri is not None:
            # send request pad to all drones that are swarming
            msg = ControllerCommand()
            msg.uri = landing_uri
            msg.data = 'return'
            self.command_pub.publish(msg)
            # set the drone pad request variable to true
            self.requested_pad[landing_uri] = True
        
        return
    
    def drone_requested_pad(self, msg):
        # set the requested pad variable to true
        self.requested_pad[msg.data] = True


    #################### Request a charged drone functions ####################

    def request_charged_drone(self):
        # self.get_logger().info('requesting charged drone')
        take_off_uri = None

        # see which drones are available (waiting and available for take off)
        # make sure that no drone is requested twice
        for key in self.drone_states:
            if self.drone_states[key] == 'waiting' and not self.request_take_off[key]:
                take_off_uri = key
                break
            
        # check if a take off uri is found
        if take_off_uri is not None:
            # send take off command to drone
            msg = ControllerCommand()
            msg.uri = take_off_uri
            msg.data = 'take off'
            self.command_pub.publish(msg)

            # a pad can  be requested again
            self.requested_pad[take_off_uri] = False
            self.request_take_off[take_off_uri] = True

    def terminate_callback(self, msg):
        if msg == "kill all":
            self.destroy_node()
            sys.exit()


def main(args=None):
    rclpy.init(args=args)
    controller = ControllerMain()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()