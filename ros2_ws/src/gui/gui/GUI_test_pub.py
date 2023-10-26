#!/usr/bin/env python3
"""
Node that generates random drone data to test out the GUI
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from topic_interface.msg import StringList
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

import random

class PublisherTest(Node):
    def __init__(self):
        super().__init__('publisher_test', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.uris1 = [
            'radio://1/60/2M/247E000001',
        ]

        self.uris2 = [
            'radio://0/90/2M/247E000002',
            'radio://0/90/2M/247E000003',
            'radio://0/90/2M/247E000004',
            'radio://0/90/2M/247E000005',
            'radio://0/90/2M/247E000006',
            'radio://0/90/2M/247E000007',
            'radio://0/90/2M/247E000008',
        ]

        

        self.drone_states = [0]*8
        self.states = [
            "initialising",
            "starting",
            "waiting",
            "kalman_reset",
            "taking off",
            "swarming",
            "landing",
            "shutdown",
            "charging",
            "error",
            "emergency_land"
        ]

        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
        self.drone_pubs = {}
        for uri in self.uris1 + self.uris2:
            self.drone_pubs[uri] = self.create_publisher(String, 'E' + uri.split('/')[-1] + '/state', 10)
        
        self.self_uris_pub1 = self.create_publisher(StringList, f"ID1/self_uris", qos_profile=latching_qos)
        self.response_pub1 = self.create_publisher(StringList, f"ID1/response", qos_profile=latching_qos)
        self.self_uris_pub1.publish(StringList(sl=self.uris1))

        self.self_uris_pub2 = self.create_publisher(StringList, f"ID0/self_uris", qos_profile=latching_qos)
        self.response_pub2 = self.create_publisher(StringList, f"ID0/response", qos_profile=latching_qos)
        self.self_uris_pub2.publish(StringList(sl=self.uris2))

        self.drone_parameters_pub1 = self.create_publisher(StringList, f"ID1/drone_parameters", 10)
        self.drone_parameters1 = ["0/0/0/0/0/0/0/0/0"]*len(self.uris1)

        self.drone_parameters_pub2 = self.create_publisher(StringList, f"ID0/drone_parameters", 10)
        self.drone_parameters2 = ["0/0/0/0/0/0/0/0/0"]*len(self.uris2)

        self.timer = self.create_timer(0.2, self.timer_callback)
    
    def timer_callback(self):
        drone = random.randint(0, 6)
        self.drone_states[drone] = (self.drone_states[drone] + 1) % len(self.states)
        if drone == 0:
            drone_param_list = [random.randint(0, 4), random.random()*1.17+3, *[random.random() for _ in range(7)]]
            self.drone_parameters1[drone] = "/".join([str(x) for x in drone_param_list])

            self.drone_pubs[self.uris1[drone]].publish(String(data=self.states[self.drone_states[drone]]))
            self.drone_parameters_pub1.publish(StringList(sl=self.drone_parameters1))
        else:
            drone_param_list = [random.randint(0, 4), random.random()*1.17+3, *[random.random() for _ in range(7)]]
            self.drone_parameters2[drone-1] = "/".join([str(x) for x in drone_param_list])

            self.drone_pubs[self.uris2[drone-1]].publish(String(data=self.states[self.drone_states[drone-1]]))
            self.drone_parameters_pub2.publish(StringList(sl=self.drone_parameters2))


def main(args=None):
    rclpy.init(args=args)
    node = PublisherTest()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()