import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from std_msgs.msg import String
from topic_interface.msg import StringList, PosVel, PosVelList

class RollingList():
    def __init__(self, size, init_values=None):
        self._size = size
        self.data = np.array([init_values]*size)
    
    def append(self, value):
        self.data = np.roll(self.data, 1, axis=0)
        self.data[0] = value
    
    def __getitem__(self, index):
        return self.data[index]

class RollingAverage():
    def __init__(self, size):
        self._size = size
        self._data = []
        self._average = 0

    def add(self, value):
        self._data.append(value)
        if len(self._data) > self._size:
            self._data.pop(0)
    
    def get(self):
        return sum(self._data)/len(self._data)

class SwarmController():
    def __init__(self, node:Node, num_radios:int):
        self.node = node
        self.num_radios = num_radios

        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.drone_subs = {}
        self.drone_uris = []
        self.drone_states = {}
        self.drone_positions = {}

        self.publish_posvel = {}

        self.swarming_uris = []

        self.drone_uris_subs = []
        self.drone_parameters_subs = []
        for i in range(self.num_radios):
            self.drone_uris_subs.append(self.node.create_subscription(StringList, f'ID{i}/self_uris', lambda msg, radio=i: self._update_drone_uris(msg, radio), qos_profile=latching_qos))
            self.drone_parameters_subs.append(self.node.create_subscription(StringList, f'ID{i}/drone_parameters', lambda msg, radio=i: self._update_drone_parameters(msg, radio), 10))
            self.drone_uris.append(dict())
        
        self.posvel_commander = self.node.create_publisher(PosVelList, 'posvel_target', 10)
        self.GUI_command_pub = self.node.create_publisher(String, 'GUI_command', qos_profile=latching_qos)
    
    def _update_drone_states(self, msg, uri):
        """
        Called when a drone changes state.
        """
        self.drone_states[uri] = msg.data

        if msg.data == 'swarming' and uri not in self.swarming_uris:
            self.swarming_uris.append(uri)
        elif msg.data != 'swarming' and uri in self.swarming_uris:
            self.swarming_uris.remove(uri)

    def _update_drone_parameters(self, msg, radio):
        """
        Called when drone parameters are received.
        """
        if len(self.drone_uris[radio]) > 0:
            for i, param in enumerate(msg.sl):
                # drone parameters: x/y/z//vx/vy/vz/yaw//battery state/battery voltage/lighthouse active/sys_isFlying/sys_isTumbled
                posvel, _ = [x.split("/") for x in param.split("//")]
                uri = self.drone_uris[radio][i]

                self.drone_positions[uri] = [float(posvel[0]), float(posvel[1]), float(posvel[2])]

    def _update_drone_uris(self, msg, radio):
        """
        Called when a radio publishes the list of drones it is connected to.
        """
        # if a drone uri is not yet there, add the uri to the dict
        for i, uri in enumerate(msg.sl):
            self.drone_uris[radio][i] = uri

            if uri not in self.drone_subs:
                self.drone_subs[uri] = self.node.create_subscription(String, 'E' + uri.split('/')[-1] + '/state', lambda msg, uri_i=uri: self._update_drone_states(msg, uri_i), 10)
                self.drone_states[uri] = 'initialising'

    def get_swarming_uris(self) -> list:
        """
        Get a list of uris of all drones that are currently swarming.
        """
        return self.swarming_uris

    def get_swarming_positions(self) -> list:
        """
        Get a list of positions of all drones that are currently swarming.
        """
        return [self.drone_positions[uri] for uri in self.swarming_uris]

    def add_drone(self):
        """
        Add a drone to the swarm. Same as pressing the add button in the GUI.
        Add a pause between consecutive calls.
        """
        self.GUI_command_pub.publish(String(data='add drone'))
    
    def remove_drone(self):
        """
        Remove a drone from the swarm. Same as pressing the remove button in the GUI.
        Add a pause between consecutive calls.
        """
        self.GUI_command_pub.publish(String(data='remove drone'))
    
    def set_position(self, uri, pos):
        """
        Set the position of a drone. This position is only sent to the drone when send_commands() is called.
        """
        self.publish_posvel[uri] = PosVel(uri=uri, vec=pos, mode="position")
    
    def set_velocity(self, uri, vel):
        """
        Set the velocity of a drone. This velocity is only sent to the drone when send_commands() is called.
        """
        self.publish_posvel[uri] = PosVel(uri=uri, vec=vel, mode="velocity")
    
    def send_commands(self):
        """
        Send all position and velocity commands to the drones.
        """
        self.posvel_commander.publish(PosVelList(data=list(self.publish_posvel.values())))
    
    def clear_commands(self):
        """
        Clear all position and velocity commands. The drones will continue with their current position and velocity.
        """
        self.publish_posvel = {}
    
    def stop_drones(self):
        """
        Stop all drones at their current position.
        """
        self.clear_commands()
        for uri in self.swarming_uris:
            self.set_position(uri, self.drone_positions[uri])
        self.send_commands()
    
