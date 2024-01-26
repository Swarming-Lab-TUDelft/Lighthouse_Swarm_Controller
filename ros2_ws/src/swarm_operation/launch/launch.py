import launch
import launch_ros.actions
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

from ament_index_python.packages import get_package_share_directory

import os
import json
import time

# read settings from configuration file
from swarm_operation.config import *

def generate_launch_description():
    if CA_MODE != "off":
        ######### generate CAConfig.json #########

        ca_template_dir = os.path.join(os.getcwd(), 'src', 'swarm_operation', 'ca_template.json')

        with open(ca_template_dir, 'r') as f:
            ca_template = json.load(f)

        ca_template[2]["position"]["z"] = ABS_BOUNDS[0][0]
        ca_template[5]["position"]["z"] = ABS_BOUNDS[0][1]

        ca_template[3]["position"]["x"] = ABS_BOUNDS[1][0]
        ca_template[4]["position"]["x"] = ABS_BOUNDS[1][1]

        ca_template[7]["position"]["y"] = ABS_BOUNDS[2][0]
        ca_template[6]["position"]["y"] = ABS_BOUNDS[2][1]

        if CA_CONFIG_DIR == "Linux":
            ca_config_dir = os.path.join(os.getcwd(), '..', 'collision_avoidance', 'LinuxBuild', 'LinuxCA_Data', 'CAConfig.json')
        else:
            ca_config_dir = os.path.join(CA_CONFIG_DIR, 'CAConfig.json')
        
        with open(ca_config_dir, 'w') as f:
            json.dump(ca_template, f, indent=0)
    
    ######### generate launch description #########

    launch_description = []

    # set logger level
    launch_description.append(launch.actions.DeclareLaunchArgument(
        "log_level",
        default_value=[LOG_LEVEL],
        description="Logging level"
        )
    )
    log_level = launch.substitutions.LaunchConfiguration("log_level")

    # Get user inputs
    # generate list of URI's
    all_uris = []
    radio_uris = [[]]
    radio_id = 0

    for drone_number in range(START_IDX_CFS, NUM_CFS+START_IDX_CFS):
        # generate URI, e.g. 'radio://0/80/2M/247E000001'
        uri = 'radio://' + str(radio_id) + '/' + str(RADIO_CHANNELS[(drone_number-1)//CFS_PER_RADIO]) + '/2M/247E' + '0'*(6-len(str(drone_number))) + str(drone_number)
        print(uri)
        radio_uris[radio_id].append(uri)
        all_uris.append(uri)

        if drone_number%CFS_PER_RADIO == 0:
            radio_uris.append([])
            radio_id += 1
    
    if len(radio_uris[-1]) == 0:
        radio_uris.pop(-1)
    NUM_RADIOS = len(radio_uris)

    ca_params = dict()
    pos_comm_params = {'number_radios': NUM_RADIOS}
    for i, j in enumerate(all_uris):
        ca_params[j] = str(i+1)
        pos_comm_params[j] = "initialising"

    
    if CA_MODE != "off":
        ca_nodename = "CollisionAvoidance"
        # launch collision avoidance node
        launch_description.append(launch_ros.actions.Node(
            package='swarm_operation',
            executable='CollisionAvoidance',
            name=ca_nodename,
            parameters=[ca_params],
            arguments=[
            "--ros-args",
            "--log-level",
            [f"{ca_nodename}:=", log_level]]
            )
        )   
    

    # launch main controller
    ctrl_nodename = "Controller"
    launch_description.append(launch_ros.actions.Node(
        package='swarm_operation',
        executable='MainController',
        name=ctrl_nodename,
        parameters=[{'number_radios': NUM_RADIOS}],
        arguments=[
            "--ros-args",
            "--log-level",
            [f"{ctrl_nodename}:=", log_level]]
        )
    )

    # launch charge manager
    pdmngr_nodename = "PadManager"
    launch_description.append(launch_ros.actions.Node(
        package='swarm_operation',
        executable='PadManager',
        name=pdmngr_nodename,
        parameters=[{'number_radios': NUM_RADIOS}],
        arguments=[
            "--ros-args",
            "--log-level",
            [f"{pdmngr_nodename}:=", log_level]]
        )
    )
    
    # launch radio handler
    for radio in range(NUM_RADIOS):
        rdhdlr_nodename = "RadioHandler" + str(radio)
        launch_description.append(launch_ros.actions.Node(
            package='swarm_operation',
            executable='RadioHandler',
            name= rdhdlr_nodename,
            parameters=[
                {'uris': radio_uris[radio]},
                {'devid': radio}],
            arguments=[
                "--ros-args",
                "--log-level",
                [f'{rdhdlr_nodename}:=', log_level]]
            )
        )
    
    # launch GUI node
    launch_description.append(IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(get_package_share_directory("gui"), "launch/GUI.launch")
            ),
            launch_arguments = {'number_radios': str(NUM_RADIOS),
                                'pattern_gui': str(PATTERN_GUI)}.items(),
                                
        )
    )
    
    # launch pos_command node
    pscmd_nodename = "PosCommand"
    launch_description.append(launch_ros.actions.Node(
        package='swarm_operation',
        executable=COMMANDER,
        name=pscmd_nodename,
        parameters=[pos_comm_params],
        arguments=[
            "--ros-args",
            "--log-level",
            [f"{pscmd_nodename}:=", log_level]]
        )
    )

    # start all the Crazyfie nodes
    for uri in all_uris:
        cf_nodename = "Drone" + uri[-2:]
        launch_description.append(launch_ros.actions.Node(
            package='swarm_operation',
            executable='CrazyflieNode',
            name=cf_nodename,
            output='screen',
            emulate_tty=True,
            parameters=[
                {'uri': j},
                {'radio_id': int(uri[8])}],
            arguments=[
                "--ros-args",
                "--log-level",
                [f"{cf_nodename}:=", log_level]]
            )
        )
    
    return launch.LaunchDescription(launch_description)
