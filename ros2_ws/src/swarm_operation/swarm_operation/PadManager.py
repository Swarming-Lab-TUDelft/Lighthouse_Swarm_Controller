"""
Node to manage charging pads. Handles charging pad requests from drones and publishes pad locations to drones.
"""


import sys
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from topic_interface.msg import StringList, Location
from std_msgs.msg import String

# read settings from configuration file
from .config import MIN_PAD_DIST


class PadManager(Node):
    def __init__(self):
        super().__init__('pad_manager', automatically_declare_parameters_from_overrides=True)

        self.num_radios = self.get_parameter('number_radios').value

        self.drone_subs = {}    # drone state subscribers
        self.drone_uris = []    # uri lists from radio nodes
        self.drone_states = {}  # current states of drones
        self.drone_pos = {}     # current position of drones

        self.charging_pads = {} # dict -> key: pad position, value: [status, uri]
                                # status: 'charging', 'landing', 'claimed', 'available', 'error'
        self.pads_in_use = []   # list of pad positions that are currently in use (landing, check charging)
        self.pads_in_queue = [] # pads that are claimed by drones but due to proximity to other pads, cannot be used yet

        # create latching qos profile
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    

        # subscribers
        self.pad_location_sub = self.create_subscription(Location, 'init_pad_location', self.init_pad_cb, 10)
        self.request_pad_sub = self.create_subscription(String, "cf_charge_req", self.req_pad_cb, 10)
        self.GUI_command_sub = self.create_subscription(String, 'GUI_command', self.GUI_command_callback, 10)
        
        self.drone_uris_subs = []
        self.drone_parameters_subs = []
        for i in range(self.num_radios):
            self.drone_uris_subs.append(self.create_subscription(StringList, f'ID{i}/self_uris', lambda msg, radio=i: self.update_drone_uris(msg, radio), qos_profile=latching_qos))
            self.drone_parameters_subs.append(self.create_subscription(StringList, f'ID{i}/drone_parameters', lambda msg, radio=i: self.update_drone_parameters(msg, radio), 10))
            self.drone_uris.append(dict())

        # publishers
        self.loc_pub = self.create_publisher(Location, 'pad_location', 10)

        self.queue_timer = self.create_timer(1.0, self.queue_timer_cb)

    ############################# Drone data callbacks #############################

    def update_drone_parameters(self, msg, radio):
        """
        Get current drone positions.
        """
        if len(self.drone_uris[radio]) > 0:
            for i, param in enumerate(msg.sl):
                posvel, _ = [x.split("/") for x in param.split("//")]
                uri = self.drone_uris[radio][i]
                self.drone_pos[uri] = (posvel[0], posvel[1], posvel[2])

    def update_drone_uris(self, msg, radio):
        """
        Get list of drone uris from radio nodes and create subscribers for each drone.
        """
        for i, uri in enumerate(msg.sl):
            self.drone_uris[radio][i] = uri

            if uri not in self.drone_subs:
                self.drone_subs[uri] = self.create_subscription(String, 'E' + uri.split('/')[-1] + '/state', lambda msg, uri_i=uri: self.update_drone_states(msg, uri_i), 10)

    def update_drone_states(self, msg, uri):
        """
        Update drone states and charging pad states based on current and previous states.
        """
        for pos, (status, uri_i) in self.charging_pads.items():
            if uri_i == uri:
                if msg.data == 'landing':
                    if self.charging_pads[pos][0] == 'claimed':
                        self.pads_in_queue.remove(pos)
                    self.charging_pads[pos][0] = 'landing'
                elif msg.data == 'charging':
                    self.charging_pads[pos][0] = 'charging'
                elif msg.data == 'swarming':
                    self.charging_pads[pos] = ['available', '']
                elif msg.data == 'land in place' and self.drone_states[uri] not in ('landing', 'taking off'):
                    self.charging_pads[pos] = ['available', '']
                elif msg.data == 'error':
                    if self.charging_pads[pos][0] not in ('charging', 'landing', 'error'):
                        self.charging_pads[pos] = ['available', '']
                    else:
                        self.charging_pads[pos][0] = 'error'
                elif msg.data == 'error handling' and self.drone_states[uri] == 'returning':
                    self.charging_pads[pos] = ['available', '']

        self.pads_in_use = [pos for pos, (status, uri_i) in self.charging_pads.items() if status == 'landing']
        
        self.drone_states[uri] = msg.data
    
    def init_pad_cb(self, msg):
        """
        Store charging pad locations.
        """
        for key in self.charging_pads:
            if np.linalg.norm(np.array(key) - np.array(msg.location)) < 0.1:
                self.get_logger().info("Pad already initialised")
                return

        self.charging_pads[tuple(msg.location)] = ['charging', msg.uri]
    
    ############################# Helper functions #############################

    def publish_position(self, pos, uri, clear):
        """
        Publish charging pad location to drone.
        """
        msg = Location()
        msg.location = list(pos)
        msg.uri = str(uri)
        msg.clear = clear
        self.loc_pub.publish(msg)

    ############################# Charging pad request #############################

    def req_pad_cb(self, msg):
        """
        Handle charging pad requests from drones.
        """
        uri = msg.data

        free_pos = None
        
        for pos, (status, uri_i) in self.charging_pads.items():
            if uri_i == uri:
                self.publish_position(pos, uri, True)
                return
            if status == 'available':
                free_pos = pos
                if len(self.pads_in_use) == 0 or np.all(np.linalg.norm(np.array(pos) - np.array(self.pads_in_use), axis=1) > MIN_PAD_DIST):
                    self.charging_pads[pos] = ['landing', uri]
                    self.publish_position(pos, uri, True)
                    return

        if free_pos is not None:
            self.charging_pads[free_pos] = ['claimed', uri]
            self.pads_in_queue.append(free_pos)
            self.publish_position(free_pos, uri, False)
            return
        
        self.get_logger().info(f'No pads available for {uri}')
    
    def queue_timer_cb(self):
        """
        Check if pads in queue can be used (check proximity to pads where a drone is landing).
        """
        if len(self.pads_in_queue) > 0:
            for pos in self.pads_in_queue.copy():
                uri = self.charging_pads[pos][1]
                if len(self.pads_in_use) == 0 or np.all(np.linalg.norm(np.array(pos) - np.array(self.pads_in_use), axis=1) > MIN_PAD_DIST):
                    self.pads_in_queue.remove(pos)
                    self.charging_pads[pos][0] = 'landing'
                    self.publish_position(pos, uri, True)
                    return
                # if pads in close proximity are still in use, check for other pads
                for pos_i, (status, uri_i) in self.charging_pads.items():
                    if status == 'available':
                        if np.all(np.linalg.norm(np.array(pos_i) - np.array(self.pads_in_use), axis=1) > MIN_PAD_DIST):
                            self.charging_pads[pos] = ['available', '']
                            self.charging_pads[pos_i] = ['landing', uri]
                            self.publish_position(pos_i, uri, True)
                            return


    ############################# Terminate #############################

    def GUI_command_callback(self, msg):
        """
        Terminate node.
        """
        if msg.data == "terminate/kill all":
            self.destroy_node()
            sys.exit()


def main(args=None):
    rclpy.init(args=args)
    node = PadManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()