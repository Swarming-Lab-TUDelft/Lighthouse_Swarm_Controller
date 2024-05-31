#!/usr/bin/env python3
"""
Simple GUI to allow switching between patterns
"""

import sys
import os
from threading import Thread
from queue import Queue, Empty
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String

from PIL import Image, ImageTk
from tkinter import PhotoImage

from GUI_theme import *
from GUI import custom_swarm_commands


command_queue = Queue()


class PatternGUINode(Node):
    def __init__(self):
        super().__init__('GUINode', automatically_declare_parameters_from_overrides=True)

        self.startup_time = time.time()

        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # publishers
        self.GUI_command_pub = self.create_publisher(String, 'GUI_command', qos_profile=latching_qos)
        #NEW
        self.ROS_pattern_switch_pub = self.create_publisher(String, '/ROS_pattern_switch', qos_profile=latching_qos)
        #NEw

        self.update_timer = self.create_timer(0.1, self.check_queue)

    def check_queue(self):
        try:
            command = command_queue.get_nowait()
            self.get_logger().info(command)
            self.GUI_command_pub.publish(String(data=command))
            #NEW
            self.ROS_pattern_switch_pub.publish(String(data=command))
            #NEW
            if command == "terminate/kill all":
                raise SystemExit
        except Empty:
            pass
            

class PatternGUI():
    def __init__(self, logger):
        self.logger = logger

        self.root = window("Pattern Selection")
        self.root.geometry("1920x1080")

        # Make the window borderless and set it to fullscreen mode
        self.root.wm_attributes('-fullscreen', 1)
        self.root.wm_attributes('-zoomed', 1)

        # Create a LabelFrame to contain the buttons
        button_frame = LabelFrame(self.root, "", padding=0)
        button_frame.pack(fill="both", expand=True)
        
        # Create a list to hold the buttons
        buttons = []
        # List of colours
        colors = ["red", "blue", "yellow", "green2"]
        images = ["Diamond.png", "VLines.png", "HLines.png", "Velocity.png"]
        text = ["Rotating Diamond", "Vertical Lines", "Horizontal Lines", "Random Velocity"]
        cmds = ["activate_rotating_diamond", "activate_ver_rotating_lines", "activate_hor_rotating_lines", "activate_vel_commander"]
        # Create buttons based on the available commands
        for idx, _ in enumerate(cmds):
            img_path = os.path.join(os.getcwd(), "src", "gui", "images", images[idx])
            button = RoundedButton(
                button_frame,
                text[idx],  # Button text
                image_path = img_path,
                color=colors[idx],  # Button color
                width=850,
                height=450,
                command=lambda i=idx: command_queue.put("custom/Patterns/" + cmds[i])
            )
            buttons.append(button)
            button.grid(row=idx// 2, column=idx % 2, padx=0, pady=0)

        # Configure row and column weights to distribute the buttons equally
        for idx, _ in enumerate(custom_swarm_commands["Patterns"]):
            button_frame.grid_rowconfigure(idx // 2, weight=1)
            button_frame.grid_columnconfigure(idx % 2, weight=1)
        
        # Bind the Escape key to the exit_app function
        self.root.bind('<Escape>', self.exit_app)
        self.root.mainloop()

    # Function to exit the application when the Escape key is pressed
    def exit_app(self, event):
        self.root.quit()

        
def main(args=None):
    rclpy.init(args=args)
    guinode = PatternGUINode()
    logger = guinode.get_logger()
    
    def spin_com_node():
        try:
            rclpy.spin(guinode)
        except SystemExit:
            pass
    
    com_node_thread = Thread(target=spin_com_node)
    com_node_thread.start()

    gui = PatternGUI(logger)

    rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()


if __name__ == '__main__':
    main()