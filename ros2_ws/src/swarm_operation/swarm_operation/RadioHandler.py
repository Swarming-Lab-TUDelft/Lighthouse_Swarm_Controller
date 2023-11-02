#!/usr/bin/env python3

"""
Node for handling the radio communication to the drones for one radio.
"""


import rclpy
from rclpy.node     import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

import sys
from threading import Thread
import copy

import cflib.crtp
from cflib.crazyflie import logger as cf_logger  # used for changing the logger level
from cflib.crtp.radiodriver import RadioManager
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import Swarm
from cflib.crtp.crtpstack import CRTPPacket

from std_msgs.msg import String
from topic_interface.msg import StringList

from .logger import Logger


class RadioHandler(Node):
    def __init__(self):
        self.initialised = False
        super().__init__('RadioHandler', automatically_declare_parameters_from_overrides=True)

        # get parameters
        self.devid = self.get_parameter('devid').value
        self.uris = self.get_parameter('uris').value
        self.get_logger().info(f"URIs: {self.uris}")

        # latching qos profile
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # subscribers
        self.GUI_command_sub = self.create_subscription(String, 'GUI_command', self.GUI_command_callback, 10)
        self.drone_subscriptions = {}
        self.drone_subs = {}
        self.drone_states = {}
        for uri in self.uris:
            self.drone_subscriptions[uri] = self.create_subscription(String, 'E' + uri.split('/')[-1] + '/command', lambda msg, uri_i=uri: self.cmd_handler(msg, uri_i), qos_profile=latching_qos)
            self.drone_subs[uri] = self.create_subscription(String, 'E' + uri.split('/')[-1] + '/state', lambda msg, uri_i=uri: self.update_drone_states(msg, uri_i), 10)
        self.controller_announcement_sub = self.create_subscription(String, 'controller_announcement', self.controller_announcement_cb, 10)

        # publishers
        self.radio_state_pub = self.create_publisher(String, f"ID{self.devid}/radio_state", 10)
        self.radio_msgs_pub = self.create_publisher(String, f"ID{self.devid}/radio_msgs", 10)
        self.self_uris_pub = self.create_publisher(StringList, f"ID{self.devid}/self_uris", qos_profile=latching_qos)
        self.drone_response_pub = self.create_publisher(StringList, f"ID{self.devid}/response", qos_profile=latching_qos)
        self.drone_parameters_pub = self.create_publisher(StringList, f"ID{self.devid}/drone_parameters", 10)

        # logger
        self.log = Logger(self.get_logger(), self.radio_msgs_pub, 'info')

        # Initialize radio drivers
        cflib.crtp.init_drivers()

        # Declare swarm instance
        self.factory = CachedCfFactory(rw_cache=f'./radio_cache/cache{self.devid}')
        self.swarm = Swarm(self.uris, factory=self.factory)

        # Publish initial values
        self.radio_state_pub.publish(String(data="initialising"))
        self.self_uris_pub.publish(StringList(sl=self.uris))
    
        # dictionary to map uris to indices
        self.uri_idx = dict([(uri, i) for i, uri in enumerate(self.uris)])
        
        # dictionary to store which drones have been rebooted
        self.rebooted = dict([(uri, False) for uri in self.uris])

        # connection variables
        self.disconnected_uris = []
        self.open_links_threads = []
        self.drone_responses = ["initialising"]*len(self.uris)

        # parameter logging
        #   add log blocks AFTER 'system state' to read more parameters:
        #       name: ((param1, type1), (param2, type2), ...)
        #       parameter name and type can be found at https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/
        #       taken from the bitcraze website: The maximum length for a log packet is 26 bytes. This, for for example, allows to log 6 floats and one uint16_t (6*4 + 2 bytes) in a single packet.
        self.param_logs_dict = {
            "pos vel": (
                ("stateEstimate.x", "float"),
                ("stateEstimate.y", "float"),
                ("stateEstimate.z", "float"),
                ("stateEstimate.vx", "float"),
                ("stateEstimate.vy", "float"),
                ("stateEstimate.vz", "float")
            ),
            "system state": (
                ("pm.state", "float"),
                ("pm.vbat", "float"),
                ("lighthouse.bsActive", "float"),
                ("supervisor.info", "uint16_t")
            )
        }
        self.parameter_configs = dict([(param, {}) for param in self.param_logs_dict.keys()])
        self.parameter_callbacks = copy.deepcopy(self.parameter_configs)
        self.param_wait_for_response = copy.deepcopy(self.parameter_configs)
        self.drone_parameters = ["//".join(["/".join(["0" for _ in self.param_logs_dict[name]]) for name in self.param_logs_dict.keys()])]*len(self.uris)

        # dictionary to map drone commands to functions
        self.command_dict = {
            "param log": self.start_param_logging,
            "stop param log": self.stop_param_logging,
            "velocity": self.send_velocity,
            "position": self.send_position,
            "hover": self.send_hover_point,
            "led": self.set_led,
            "stop motors": self.stop_motors,
            "reboot": self.reboot
        }

        # timer to check if radio is attached (only running at startup)
        self.radio_attached_timer = self.create_timer(1, self.radio_attached)


    ############################# Startup Functions #############################

    def radio_attached(self):
        """
        Check if a radio is attached.
        """
        try:
            _ = RadioManager.open(self.devid)
            self.radio_attached_timer.destroy()
            self.radio_state_pub.publish(String(data="radio found"))
        except Exception:
            self.radio_state_pub.publish(String(data=f"trying again"))
    
    def connect_to_drones(self):
        """
        Try to connect to all drones in self.uris.
        """
        self.radio_state_pub.publish(String(data="connecting to drones"))
        self.log.info("connecting to drones")
        self.drone_responses = ["connecting"]*len(self.uris)
        self.drone_response_pub.publish(StringList(sl=self.drone_responses))

        # add disconnect callback to each drone
        for scf in self.swarm._cfs.values():
            scf.cf.connection_lost.add_callback(self.disconnect_callback)
            cf_logger.setLevel(35)
        
        # Connect to drones
        self.open_links()

        for i, response in enumerate(self.drone_responses):
            if response != "disconnected":
                self.drone_responses[i] = "connected"

        self.drone_response_pub.publish(StringList(sl=self.drone_responses))
        self.radio_state_pub.publish(String(data="ready"))
        self.log.info("Radio initialised")
        self.initialised = True
    
    ############################# Callbacks #############################
    
    def controller_announcement_cb(self, msg):
        """
        Called when the main controller publishes on the controller_announcement topic.
        """
        if msg.data == 'radios found':
            self.connect_to_drones()
    
    def update_drone_states(self, msg, uri):
        """
        Called when a drone changes its state.
        """
        self.drone_states[uri] = msg.data
        
    
    ############################# Helper Functions #############################

    def open_links(self):
        """
        Open links to all drones in self.uris
        """
        open_links_threads = []
        for uri, scf in self.swarm._cfs.items():
            thread = Thread(target=self._thread_function_open_links, args=(scf, uri))
            open_links_threads.append(thread)
            thread.start()

        for i, thread in enumerate(open_links_threads):
            thread.join(timeout=15)
            if thread.is_alive():
                self.log.info(f"Error: {self.uris[i]} connection timed out")
                self.disconnect_callback(list(self.swarm._cfs.keys())[i], None)

    def _thread_function_open_links(self, *args):
        """
        Actual function to open a link to a drone. Is called multiple times in parallel.
        """
        try:
            args[0].open_link()
        except Exception as e:
            self.log.info(f"Error: {args[1]} connection failed: {e}")
            if args[1] not in self.disconnected_uris:
                self.disconnected_uris.append(args[1])
            self.drone_responses[self.uri_idx[args[1]]] = "disconnected"
    
    def empty_error_cb(self, msg):
        """
        Used to disable disconnect callback when rebooting.
        """
        pass

    def log_active_cb(self, log, started, config, uri, param):
        """
        Called when a log is started or stopped.
        """
        # forward response to the drone
        self.update_drone_responses(uri, f"logcb:{config.name.split('|')[-1]}:{int(started)}")
        # if a log is deleted
        if not started:
            config.started_cb.remove_callback(self.parameter_callbacks[param][uri])
            if uri in self.parameter_callbacks[param]:
                log.log_blocks.remove(log._find_block(config.id))
                self.parameter_callbacks[param].pop(uri)
                self.parameter_configs[param].pop(uri)

    
    ############################# ROS Callbacks #############################
    
    def update_drone_responses(self, uri, response):
        """
        Publish command responses.
        """
        self.drone_responses[self.uri_idx[uri]] = response
        self.drone_response_pub.publish(StringList(sl=self.drone_responses))
        
    def cmd_handler(self, msg, uri):
        """
        Run correct function based on the command received.
        """
        if uri not in self.disconnected_uris:
            command = msg.data.split("/")
            if command[0] in self.command_dict:
                self.command_dict[command[0]](uri, *command[1:])
        else:
            self.update_drone_responses(uri, "disconnected")
    
    def disconnect_callback(self, uri, msg):
        """
        Is called when a drone disconnects.
        """
        self.log.info(f"Error: {uri} disconnected")
        if self.initialised:
            self.update_drone_responses(uri, "disconnected")
        else:
            # if the radio is not initialised yet, set response to disconnected and terminate the thread by setting the connect event
            self.drone_responses[self.uri_idx[uri]] = "disconnected"
            self.swarm._cfs[uri]._connect_event.set()

        if uri not in self.disconnected_uris:
                self.disconnected_uris.append(uri)
    
    def GUI_command_callback(self, msg):
        """
        Terminate this node when the GUI sends a terminate message.
        """
        if msg.data == "e stop":
            self.get_logger().info("EMERGENCY STOP TRIGGERED")
            for idx in self.uri_idx.keys():
                self.stop_motors(idx)
                self.swarm.close_links()
                self.destroy_node()
                sys.exit()
        if msg.data == "terminate/kill all":
            self.swarm.close_links()
            self.destroy_node()
            sys.exit()

    
    ############################# Commands #############################
    
    def start_param_logging(self, uri, param, period):
        """
        Start logging a parameter.
        """
        period = float(period)
        cf = self.swarm._cfs[uri].cf

        # if the drone was rebooted, enable the disconnect callback again
        if self.rebooted[uri]:
            cf.link._thread._link_error_callback = cf._link_error_cb
            self.rebooted[uri] = False

        # if param is invalid
        if param not in self.param_logs_dict:
            self.log.info(f"Invalid logging parameter in start logging: {param}")
            return

        # if the crazyflie has no link
        if not cf.link:
            self.log.info(f"error in param log for {uri}: no link")
            return

        # if the parameter log already exists
        if uri in self.parameter_configs[param]:
            # if the period matches
            if self.parameter_configs[param][uri].period_in_ms == period:
                self.parameter_configs[param][uri].start()
            else:
                self.log.info(f"error in param log for {uri}: already logging {param}, delete previous log first")
            return
        
        # create logconfig
        self.parameter_configs[param][uri] = LogConfig(name=f"Log {uri}|{param}:{period}", period_in_ms=period)
        log_config = self.parameter_configs[param][uri]

        # create log callback
        cb = lambda log, started, config_i=log_config, uri_i=uri, param_i=param: self.log_active_cb(cf.log, started, config_i, uri_i, param_i)
        self.parameter_callbacks[param][uri] = cb
        log_config.started_cb.add_callback(cb)

        # add variables to logconfig
        for p, p_type in self.param_logs_dict[param]:
            log_config.add_variable(p, p_type)
        
        # add logconfig to crazyflie
        log_config.data_received_cb.add_callback(lambda timestamp, data, logconf, uri_i=uri, param_i=param: self.parameter_log_callback(timestamp, data, logconf, uri_i, param_i))
        cf.log.add_config(log_config)

        # start logging
        log_config.start()


        self.update_drone_responses(uri, f"{param} logging set")
            

    def stop_param_logging(self, uri, param):
        """
        Stop logging a parameter.
        """
        if param in self.param_logs_dict:
            if uri in self.parameter_configs[param]:
                self.parameter_configs[param][uri].delete()
                
            self.update_drone_responses(uri, f"{param} logging stopped")
        else:
            self.log.info(f"Invalid logging parameter in stop logging: {param}")
    
    def send_velocity(self, uri, vx, vy, vz, yaw):
        """
        Send velocity command.
        """
        cf = self.swarm._cfs[uri].cf

        cf.commander.send_velocity_world_setpoint(float(vx), float(vy), float(vz), float(yaw))
        self.update_drone_responses(uri, "velocity set")
        # self.log.info(f"Velocity send for {uri}: {vx}|{vy}|{vz}|{yaw}")
    
    def send_position(self, uri, x, y, z, yaw):
        """
        Send position command.
        """
        cf = self.swarm._cfs[uri].cf

        cf.commander.send_position_setpoint(float(x), float(y), float(z), float(yaw))
        self.update_drone_responses(uri, "position set")
    
    def send_hover_point(self, uri, vx, vy, yawrate, zdistance):
        """
        Send hover setpoint commnand.
        """
        cf = self.swarm._cfs[uri].cf

        cf.commander.send_hover_setpoint(float(vx), float(vy), float(yawrate), float(zdistance))
        self.update_drone_responses(uri, "hover set")
    
    def stop_motors(self, uri):
        """
        Send stop command.
        """
        cf = self.swarm._cfs[uri].cf

        cf.commander.send_stop_setpoint()
        cf.commander.send_notify_setpoint_stop()
        self.update_drone_responses(uri, "motors stopped")
    
    def set_led(self, uri, bits):
        """
        Send set led command.
        """
        cf = self.swarm._cfs[uri].cf

        cf.param.set_value('led.bitmask', bits)
        self.update_drone_responses(uri, "led set")
    
    def reboot(self, uri):
        """
        Reboot the drone.
        """
        cf = self.swarm._cfs[uri].cf

        # disable disconnect callback
        cf.link._thread._link_error_callback = self.empty_error_cb
        self.rebooted[uri] = True

        # send reboot packets
        cf.send_packet(CRTPPacket(0xFF, [0xfe, 0xFF]))
        cf.send_packet(CRTPPacket(0xFF, [0xfe, 0xF0, 1]))

        # clear log blocks
        cf.log.log_blocks = []
        for param in self.parameter_configs.keys():
            self.parameter_configs[param].pop(uri, None)
            self.parameter_callbacks[param].pop(uri, None)

        self.update_drone_responses(uri, "rebooted")


    
    ############################# Parameter callbacks #############################
    
    def parameter_log_callback(self, timestamp, data, logconf, uri, param):
        """
        Is called when a log packet is received. Forward the data to the drones.
        """
        ordered_data = []
        for p, _ in self.param_logs_dict[param]:
            ordered_data.append(data[p])
        
        current_param = self.drone_parameters[self.uri_idx[uri]].split("//")
        idx = list(self.parameter_configs.keys()).index(param)
        current_param[idx] = '/'.join(str(x) for x in ordered_data)
        self.drone_parameters[self.uri_idx[uri]] = '//'.join(current_param)

        self.drone_parameters_pub.publish(StringList(sl=self.drone_parameters))
        

############################# Main function #############################

def main(args=None):
    rclpy.init(args=args)
    node = RadioHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.swarm.close_links()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
