import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

import sys

from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from .unity_communication import UnityBridge

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
        parameters = self._parameters

        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.terminate_sub = self.create_subscription(String, 'terminate', self.terminate_callback, qos_profile=latching_qos)
        
        self.CA_command_pub = self.create_publisher(String, 'CA_command', 10)

        # update CFS and their [state, bool: drone_init]
        drone_group = MutuallyExclusiveCallbackGroup()
        for uri in parameters:
            if self.get_parameter(uri).get_parameter_value().string_value == "":
                continue
            quad = QuadCopter(None, self.get_parameter(uri).get_parameter_value().string_value)
            self.quad_dict[uri] = quad
            quad.subscription = self.create_subscription(String, 'E' + uri[16:] + '/CA', lambda msg, uri_i=uri: self.get_CA_inputs(msg, uri_i), 10, callback_group=drone_group) #10 buffer size previously

        self.nbQuads = len(self.quad_dict)
        self.cfPrefix = "cfPrefix"
        self.unityBridge = UnityBridge()
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
        
        sendstring = ""

        # 5. Send commands to drones
        for droneName in self.quad_dict:
            quad = self.quad_dict[droneName]
            uri = droneName
            quad.velocityCommand = commandList[quad.ID]
            sendstring += uri[16:] + '/' + str(quad.velocityCommand[0]) + '/' + str(quad.velocityCommand[1]) + '/' + str(quad.velocityCommand[2]) + '/'
        
        self.CA_command_pub.publish(String(data=sendstring))

        self.frameID += 1
    
    def terminate_callback(self, msg):
        if msg.data == "kill all":
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
