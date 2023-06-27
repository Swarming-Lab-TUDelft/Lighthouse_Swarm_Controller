import rclpy
from rclpy.node import Node

from tkinter import ttk
import sys
from threading import Thread
from queue import Queue, Empty
import numpy as np
import time
import copy

from .GUI_theme import *

from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String
from swarm_interface.msg import StringList, ControllerCommand

# TODO:
# - add extra drone column
# - add charging symbol


drone_params = {}
drone_states = {}
drone_msgs = {}
radios = {}
radio_msgs = {}
swarm_data = {
    "req_num_drones": 0,
    "act_num_drones": 0,
    "num_drones_landing": 0,
    "num_drones_init": 0,
    "num_drones_charging": 0,
    "num_drones_charged": 0,
    "num_drones_error": 0
}

command_queue = Queue()

def get_percentage_flying(voltage):
    return 1.54*voltage - 4.85

def get_percentage_onground(voltage):
    return 2.5*voltage - 9.5

states_flying = ("taking off", "swarming", "returning", "landing", "land in place")


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


class GUIComNode(Node):
    def __init__(self):
        super().__init__('GUINode', automatically_declare_parameters_from_overrides=True)

        self.startup_time = time.time()

        # subscriptions dict that listens to the state of each drone
        self.drone_subs = {}
        self.drone_msgs_subs = {}
        self.drone_uris = []

        self.num_radios = self.get_parameter('number_radios').value

        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.battery_level = RollingAverage(30)

        # publishers
        self.emergency_land_pub_ = self.create_publisher(String, 'emergency_land', 10)
        self.update_req_cfs_pub = self.create_publisher(String, 'req_cfs', 10)
        self.terminate_pub = self.create_publisher(String, 'terminate', qos_profile=latching_qos)
        self.return_all_pub = self.create_publisher(String, 'return_all', 10)
        self.external_state_update_pub = self.create_publisher(String, 'external_state_update', 10)
        self.return_one_pub = self.create_publisher(ControllerCommand, 'controller_command', 10)

        # subscribers
        self.drone_uris_subs = []
        self.drone_parameters_subs = []
        self.radio_state_subs = []
        self.radio_msgs_subs = []
        self.radio_response_subs = []
        for i in range(self.num_radios):
            self.drone_uris_subs.append(self.create_subscription(StringList, f'ID{i}/self_uris', lambda msg, radio=i: self.update_drone_uris(msg, radio), qos_profile=latching_qos))
            self.drone_parameters_subs.append(self.create_subscription(StringList, f'ID{i}/drone_parameters', lambda msg, radio=i: self.update_drone_parameters(msg, radio), 10))
            self.radio_state_subs.append(self.create_subscription(String, f'ID{i}/radio_state', lambda msg, radio=i: self.update_radio_states(msg, radio), 10))
            self.radio_msgs_subs.append(self.create_subscription(String, f'ID{i}/radio_msgs', lambda msg, radio=i: self.update_radio_msgs(msg, radio), 10))
            self.radio_response_subs.append(self.create_subscription(StringList, f'ID{i}/response', lambda msg, radio=i: self.update_radio_response(msg, radio), qos_profile=latching_qos))
            radios[i] = {"state": "node not launched", "uris": [], "response": []}
            self.drone_uris.append(dict())

        # dict to link user command with function
        self.command_dict = {
            "emergency land": self.emergency_land,
            "add drone": self.add_drone,
            "remove drone": self.remove_drone,
            "return all": self.return_all,
            "terminate": self.terminate,
            "formation": self.formation,
            "land in place one": self.land_in_place_one,
            "return one": self.return_one,
        }

        self.update_timer = self.create_timer(0.1, self.check_queue)

    def check_queue(self):
        try:
            command = command_queue.get_nowait()
            self.get_logger().info(command)
            command = command.split("/")
            if len(command) == 1:
                self.command_dict[command[0]]()
            else:
                self.command_dict[command[0]](*command[1:])
        except Empty:
            pass
    
    

    def update_drone_parameters(self, msg, radio):
        if len(self.drone_uris[radio]) > 0:
            for i, param in enumerate(msg.sl):
                posvel, system_state = [x.split("/") for x in param.split("//")]
                uri = self.drone_uris[radio][i]
                params = {}

                self.battery_level.add(100*np.clip(get_percentage_flying(float(system_state[1])) if drone_states[uri] in states_flying else get_percentage_onground(float(system_state[1])), 0, 1))

                params["bat_state"] = system_state[0]
                params["bat_level"] = self.battery_level.get()
                params["pos"] = (posvel[0], posvel[1], posvel[2])
                params["vel"] = (posvel[3], posvel[4], posvel[5])
                params["radio"] = radio

                drone_params[uri] = params
    
    def update_drone_uris(self, msg, radio):
        radios[radio]["uris"] = msg.sl
        for i, uri in enumerate(msg.sl):
            self.drone_uris[radio][i] = uri

            if uri not in self.drone_subs:
                self.drone_subs[uri] = self.create_subscription(String, 'E' + uri[16:] + '/state', lambda msg, uri_i=uri: self.update_drone_states(msg, uri_i), 10)
                self.drone_msgs_subs[uri] = self.create_subscription(String, 'E' + uri[16:] + '/msgs', lambda msg, uri_i=uri: self.update_drone_msgs(msg, uri_i), 10)
                
    def update_drone_states(self, msg, uri):
        drone_states[uri] = msg.data

        swarm_data["act_num_drones"] = 0
        swarm_data["num_drones_init"] = 0
        swarm_data["num_drones_charging"] = 0
        swarm_data["num_drones_charged"] = 0
        swarm_data["num_drones_landing"] = 0
        swarm_data["num_drones_error"] = 0

        for state in drone_states.values():
            if state in ("initialising", "checking pad", "starting"):
                swarm_data["num_drones_init"] += 1
            elif state == "charging":
                swarm_data["num_drones_charging"] += 1
            elif state == "waiting":
                swarm_data["num_drones_charged"] += 1
            elif state in ("returning", "landing", "check charging"):
                swarm_data["num_drones_landing"] += 1
            elif state in ("error", "disconnected", "error handling", "shutdown"):
                swarm_data["num_drones_error"] += 1
            elif state in ("swarming", "taking off"):
                swarm_data["act_num_drones"] += 1
    
    def update_drone_msgs(self, msg, uri):
        if uri not in drone_msgs:
            drone_msgs[uri] = [msg.data, ]     
        else:
            drone_msgs[uri].append(f"[{round(time.time()-self.startup_time, 2)}] {msg.data}")
    
    def update_radio_states(self, msg, radio):
        radios[radio]["state"] = msg.data
    
    def update_radio_msgs(self, msg, radio):
        if radio not in radio_msgs:
            radio_msgs[radio] = [msg.data, ]     
        else:
            radio_msgs[radio].append(f"[{round(time.time()-self.startup_time, 2)}] {msg.data}")
    
    def update_radio_response(self, msg, radio):
        radios[radio]["response"] = msg.sl

    def emergency_land(self):
        self.emergency_land_pub_.publish(String(data="Land now"))
    
    def land_in_place_one(self, uri):
        msg = String()
        msg.data = uri[16:] + "/land in place"
        self.external_state_update_pub.publish(msg)
    
    def add_drone(self):
        self.update_req_cfs_pub.publish(String(data="more"))
    
    def remove_drone(self):
        self.update_req_cfs_pub.publish(String(data="less"))
    
    def return_all(self):
        # publish return all to the topic return_all
        msg = String()
        msg.data = "return all"
        self.return_all_pub.publish(msg)
        
        # TODO: make this more robust
        # for i in range(swarm_data["act_num_drones"]):
        #     self.update_req_cfs_pub.publish(String(data="less"))
    
    def return_one(self, uri):
        msg = ControllerCommand()
        msg.uri = uri
        msg.data = "request pad"
        self.return_one_pub.publish(msg)
    
    def formation(self, form):
        pass
    
    def terminate(self):
        self.terminate_pub.publish(String(data="kill all"))
        raise SystemExit
        
        
  
class GUI():
    def __init__(self, logger):
        self.logger = logger
        self.active_uri = ""
        self.active_radio = ""

        self.red_states = ["node not launched", "error"] # states that should be displayed in red

        self.root = window("Crazyflie Swarm")
        self.root.geometry("1400x800")

        self.root.columnconfigure(0, weight=0)
        self.root.columnconfigure(1, weight=1, uniform="root1")
        self.root.columnconfigure(2, weight=1, uniform="root1")
        self.root.rowconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)
        
        ########## DRONE CARDS ##########
        self.drone_cards = {}
        self.drones_frame = LabelFrame(self.root, text="drones", padding=0, scroll=True)
        self.drones_frame.grid(row=0, column=0, sticky="nesw")

        ########## DRONE CONTROLS ##########
        self.drone_control_frame = LabelFrame(self.root, text="drone controls")
        self.drone_control_frame.grid(row=0, column=1, sticky="nsew")
        self.drone_control_frame.columnconfigure(0, weight=1)
        self.drone_control_frame.rowconfigure(0, weight=1)
        ttk.Label(self.drone_control_frame, text="No drone selected", style="grey.TLabel", anchor="center").grid(row=0, column=0, sticky="nsew")
        self.drone_control_inner_frame = None
        
        ########## RADIO CARDS ##########
        self.radio_cards = {}
        self.radios_frame = LabelFrame(self.root, text="radios", padding=0)
        self.radios_frame.grid(row=1, column=0, sticky="nsew")

        ########## RADIO INFO ##########
        self.radio_info_frame = LabelFrame(self.root, text="radio info")
        self.radio_info_frame.grid(row=1, column=1, sticky="nsew")
        self.radio_info_frame.columnconfigure(0, weight=1)
        self.radio_info_frame.rowconfigure(0, weight=1)
        ttk.Label(self.radio_info_frame, text="No radio selected", style="grey.TLabel", anchor="center").grid(row=0, column=0, sticky="nsew")
        self.radio_info_inner_frame = None

        ########## SWARM CONTROLS ##########
        # - return all to base
        # - message box
        swarm_control_frame = LabelFrame(self.root, text="swarm controls")
        swarm_control_frame.grid(row=0, column=2, rowspan=2, sticky="nsew")
        
        swarm_commands = {
            "emergency_land": lambda: command_queue.put("emergency land"),
            "add_drone": lambda: command_queue.put("add drone"),
            "remove_drone": lambda: command_queue.put("remove drone"),
            "return_all": lambda: command_queue.put("return all"),
            "formation": {
                "cc": lambda: command_queue.put("formation/cc"),
                "rp": lambda: command_queue.put("formation/rp"),
                "ch": lambda: command_queue.put("formation/ch"),
                "cv": lambda: command_queue.put("formation/cv"),
                "lc": lambda: command_queue.put("formation/lc"),
            }
        }
        self.swarm_control_inner_frame = SwarmDataFrame(swarm_control_frame, swarm_data, swarm_commands)
        self.swarm_control_inner_frame.pack(fill="both", expand=True)

        # for i, f in enumerate(font.families()):
        #     ttk.Label(swarm_control_frame, text=f, font=(f, 12)).grid(row=i+7, column=0, columnspan=2, sticky="w")

        

        self.update_drone_cards()
        self.root.after(100, self.update_all)
    
    def update_all(self):
        self.update_drone_cards()
        self.update_radio_cards()
        self.update_drone_controls()
        self.update_radio_info()
        self.update_swarm_controls()
        self.root.after(100, self.update_all)
    
    def update_drone_cards(self):
        for uri, params in copy.deepcopy(drone_params).items():
            if uri not in drone_states:
                drone_states[uri] = "node not launched"
            if uri not in self.drone_cards:
                self.drone_cards[uri] = DroneCard(self.drones_frame, uri, drone_states[uri], float(params["bat_level"]), lambda uri_i=uri: self.set_active_uri(uri_i))
                self.drone_cards[uri].pack(fill="x", expand=False, side="top", pady=2)
                SeparatorH(self.drones_frame).pack(fill="x", expand=False, side="top", pady=0)
            else:
                if drone_states[uri] == "node not launched":
                    params["bat_level"] = -1
                text_color = text_red if drone_states[uri] in self.red_states else text_white

                self.drone_cards[uri].state_label.config(text=drone_states[uri], foreground=text_color)
                self.drone_cards[uri].bar.set_progress(float(params["bat_level"]))
    
    def update_drone_controls(self):
        if self.active_uri not in drone_msgs:
            drone_msgs[self.active_uri] = ["",]
        if self.active_uri != "":
            if not self.drone_control_inner_frame:
                self.drone_control_inner_frame = DroneDataFrame(
                                                                self.drone_control_frame,
                                                                self.active_uri,
                                                                drone_states[self.active_uri],
                                                                drone_params[self.active_uri],
                                                                command_el = lambda uri: command_queue.put(f"land in place one/{uri}"),
                                                                command_rl = lambda uri: command_queue.put(f"return one/{uri}"),
                                                                )
                self.drone_control_inner_frame.grid(row=0, column=0, sticky="nsew")
            else:
                self.drone_control_inner_frame.update(self.active_uri, drone_states[self.active_uri], drone_params[self.active_uri], drone_msgs[self.active_uri])

    def update_radio_cards(self):
        for i, data in radios.items():
            if i not in self.radio_cards:
                self.radio_cards[i] = RadioCard(self.radios_frame, i, data["state"], lambda radio_i=i: self.set_active_radio(radio_i))
                self.radio_cards[i].pack(fill="x", expand=False, side="top", pady=2)
                SeparatorH(self.radios_frame).pack(fill="x", expand=False, side="top", pady=0)
            else:
                text_color = text_red if data["state"] in self.red_states else text_white

                self.radio_cards[i].state_label.config(text=data["state"], foreground=text_color)
    
    def update_radio_info(self):
        if self.active_radio not in radio_msgs:
            radio_msgs[self.active_radio] = ["",]
        if self.active_radio != "":
            if not self.radio_info_inner_frame:
                self.radio_info_inner_frame = RadioDataFrame(self.radio_info_frame, self.active_radio, radios[int(self.active_radio)])
                self.radio_info_inner_frame.grid(row=0, column=0, sticky="nsew")
            else:
                self.radio_info_inner_frame.update(self.active_radio, radios[self.active_radio], radio_msgs[self.active_radio])
    
    def update_swarm_controls(self):
        self.swarm_control_inner_frame.update(swarm_data)
                
    def set_active_uri(self, uri):
        if self.active_uri == "":
            self.drone_cards[uri].active = True
            self.active_uri = uri
        if uri != self.active_uri:
            self.drone_cards[uri].active = True
            self.drone_cards[self.active_uri].active = False
            self.drone_cards[self.active_uri].on_leave()
            self.active_uri = uri
    
    def set_active_radio(self, radio):
        if self.active_radio == "":
            self.radio_cards[radio].active = True
            self.active_radio = radio
        if radio != self.active_radio:
            self.radio_cards[radio].active = True
            self.radio_cards[self.active_radio].active = False
            self.radio_cards[self.active_radio].on_leave()
            self.active_radio = radio



def main(args=None):
    rclpy.init(args=args)
    guinode = GUIComNode()
    logger = guinode.get_logger()
    
    def spin_com_node():
        try:
            rclpy.spin(guinode)
        except SystemExit:
            pass
    
    com_node_thread = Thread(target=spin_com_node)
    com_node_thread.start()

    gui = GUI(logger)
    gui.root.mainloop()
    command_queue.put("terminate")
    com_node_thread.join()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()