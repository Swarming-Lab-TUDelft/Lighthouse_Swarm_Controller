import launch
import launch_ros.actions
import math

# read settings from configuration file
from swarm_operation.config import NUM_CFS, START_IDX_CFS, CFS_PER_RADIO, RADIO_CHANNELS

def generate_launch_description():
    launch_description = []

    # generate list of URI's
    all_uris = []
    radio_uris = [[],]
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
    
    NUM_RADIOS = len(radio_uris)

    # generate list of parameters for the ORCA and pos_command nodes
    orca_params = dict()
    pos_comm_params = {'number_radios': NUM_RADIOS}
    for i, j in enumerate(all_uris):
        orca_params[j] = str(i+1)
        pos_comm_params[j] = "initialising"
    
    # launch ORCA node
    launch_description.append(launch_ros.actions.Node(
                package='swarm_operation',
                executable='CollisionAvoidance',
                name="ORCA1",
                parameters=[orca_params]))

    # launch main controller
    launch_description.append(launch_ros.actions.Node(
                package='swarm_operation',
                executable='MainController',
                name="Controller",
                parameters=[
                            {'number_radios': NUM_RADIOS}
                           ]
                ))
    
    # launch charge manager
    launch_description.append(launch_ros.actions.Node(
                package='swarm_operation',
                executable='PadManager',
                name="PadManager",
                parameters=[
                            {'number_radios': NUM_RADIOS}
                           ]
                ))
    
    # launch radio handler
    for radio in range(NUM_RADIOS):
        # print(uri_dict[str(radios)])
        launch_description.append(launch_ros.actions.Node(
                    package='swarm_operation',
                    executable='RadioHandler',
                    parameters=[
                        {'uris': radio_uris[radio]},
                        {'devid': radio}
                    ]))
    
    # launch GUI node
    launch_description.append(launch_ros.actions.Node(
                package='swarm_operation',
                executable='GUI',
                name="GUI",
                parameters=[
                            {'number_radios': NUM_RADIOS}
                           ]
                ))
    
    # launch pos_command node
    launch_description.append(launch_ros.actions.Node(
                package='swarm_operation',
                executable='PositionCommander',
                name="PosCommand",
                parameters=[pos_comm_params]))
    
    # start all the Crazyfie nodes
    for i, j in enumerate(all_uris):
        launch_description.append(launch_ros.actions.Node(
                package='swarm_operation',
                executable='CrazyflieNode',
                name="Drone_" + str(i+1),
                output='screen',
                emulate_tty=True,
                parameters=[
                    {'uri': j},
                    {'radio_id': int(j[8])},
                ]))
        
    return launch.LaunchDescription(launch_description)
