import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

import sys

from std_msgs.msg import String
from topic_interface.msg import PosVel, PosVelList

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from .unity_communication import UnityBridge

from zmq.error import ZMQError

class QuadCopter:
    def __init__(self, subscription, ID = "", position = [0.0,0.0,0.0], velocity = [0.0,0.0,0.0], desiredVelocity = [0.0,0.0,0.0], velocityCommand = [0.0,0.0,0.0], rotation = [0.0,0.0,0.0,1.0], size = [1.0,1.0,1.0]):
        self.subscription = subscription
        self.ID = ID
        self.position = position
        self.velocity = velocity
        self.desiredVelocity = desiredVelocity
        self.velocityCommand = velocityCommand
        self.rotation = rotation
        self.size = size

class CFCA(Node):
    def __init__(self):
        super().__init__('cfCA', allow_undeclared_parameters=False, automatically_declare_parameters_from_overrides=True)

        # dictionary of Crazyflies with State
        self.quad_dict = {}
        self.parameters = self._parameters

        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.drone_group = MutuallyExclusiveCallbackGroup()
        self.GUI_command_sub = self.create_subscription(String, 'GUI_command', self.GUI_command_callback, 10, callback_group=self.drone_group)
        
        self.CA_command_pub = self.create_publisher(PosVelList, 'CA_command', 10)

        self.unityBridge = None
        self.initialised = False
        self.init_timer = self.create_timer(0.5, self.initialise_bridge)

    def initialise_bridge(self):
        self.init_timer.destroy()

        # update CFS and their [state, bool: drone_init]
        for uri in self.parameters:
            if self.get_parameter(uri).get_parameter_value().string_value == "":
                continue
            quad = QuadCopter(None, self.get_parameter(uri).get_parameter_value().string_value)
            self.quad_dict[uri] = quad
            quad.subscription = self.create_subscription(String, 'E' + uri.split('/')[-1] + '/CA', lambda msg, uri_i=uri: self.get_CA_inputs(msg, uri_i), 10, callback_group=self.drone_group)
        
        self.nbQuads = len(self.quad_dict)
        self.cfPrefix = "cfPrefix"
        try:
            self.unityBridge = UnityBridge()
            self.initialised = True
        except ZMQError:
            self.get_logger().info("Unity communication address already in use: restart wsl/ubuntu and try again")
            executor.shutdown(timeout_sec=0)
            sys.exit()
        
        for quad in self.quad_dict:
            self.unityBridge.add_quadrotor(self.quad_dict[quad])

        self.get_logger().info("Sending settings to Unity")
        self.SettingsToUnity()
        self.get_logger().info("Sent settings to Unity")
        self.frameID = 1

        self.main_timer = self.create_timer(0.1, self.main_loop)

    def SettingsToUnity(self):
        self.unityBridge.connectUnity()
    
    def get_CA_inputs(self, msg, uri):
        if self.initialised:
            # split the message into a list of strings
            CA_input = [float(x) for x in msg.data.split('/')]

            # update the dictionary with the new values
            # [des vx, des vy, des vz, act x, act y, act z, act vx, act vy, act vz]
            quad = self.quad_dict[uri]
            quad.desiredVelocity = [CA_input[0], CA_input[1], CA_input[2]]
            quad.position = [CA_input[3], CA_input[4], CA_input[5]]
            quad.velocity = [CA_input[6], CA_input[7], CA_input[8]]

            if self.unityBridge.unity_ready_ == False:
                self.get_logger().info("Not connected to Unity skipping frame")
                return

            self.unityBridge.updateQuadrotor(quad)

    def main_loop(self):
        if self.unityBridge.unity_ready_ == False:
            self.get_logger().info("Not connected to Unity skipping frame")
            return

        # 3. Send updated data to Unity
        self.unityBridge.toUnity(self.frameID, self.get_logger())

        # 4. Wait for confirmation from Unity
        commandList = self.unityBridge.fromUnity(self.frameID, self.get_logger())
        
        msg_array = []

        # 5. Send commands to drones
        for droneName in self.quad_dict:
            quad = self.quad_dict[droneName]

            msg = PosVel()
            msg.uri = droneName.split('/')[-1]
            msg.vec = commandList[quad.ID]

            msg_array.append(msg)
        
        self.CA_command_pub.publish(PosVelList(data=msg_array))

        self.frameID += 1
    
    def GUI_command_callback(self, msg):
        if msg.data == "terminate/kill all":
            if self.initialised:
                self.unityBridge.disconnectUnity()
            executor.shutdown(timeout_sec=0)
            sys.exit()

executor = None
def main(args=None):
    global executor
    rclpy.init(args=args)
    node = CFCA()
    try:
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except (SystemExit, KeyboardInterrupt):
        executor.shutdown(timeout_sec=0)
