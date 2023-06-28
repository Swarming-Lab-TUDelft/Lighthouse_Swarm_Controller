import launch
import launch_ros.actions
import math

# read settings from configuration file
with open('./src/swarm_operation/launch.txt') as config:
    launch_args = config.read().splitlines()
    for arg in launch_args:
        if arg.split(':')[0] == 'Number of cfs':
            number_crazyflies = int(arg.split(':')[1])
        elif arg.split(':')[0] == 'Start index cfs':
            start_index = int(arg.split(':')[1])
        elif arg.split(':')[0] == 'Number of drones per radio':
            drones_per_radio = int(arg.split(':')[1])
        elif arg.split(':')[0] == 'Number of radios':
            number_radios = int(arg.split(':')[1])
        elif arg.split(':')[0] == 'Radio channels':
            channel_list = arg.split(':')[1].split(',')
            channels = []
            radio_index = []
            for i in range(number_radios):
                channels.append(str(int(channel_list[i])))
                radio_index.append(str(i))

# radio specifications
radio_specs = {"radio": radio_index,
               "channel": channels}

# round this number down
start_index_radios = start_index / drones_per_radio - 0.01
start_index_radios = math.floor(start_index_radios)

def generate_launch_description():
    launch_description = []

    # generate list of URI's
    all_uris = []
    uris = []
    uri_dict = dict()
    radio_index = 0
    radio_id = 0 

    for drone_number in range(start_index, number_crazyflies+start_index):
        # increase radio index and add URI's to dictionary
        if drone_number > drones_per_radio * (radio_index + 1):
            if len(uris) != 0:
                uri_dict[radio_specs["radio"][radio_id]] = uris
                uris = []
                radio_id += 1

            radio_index += 1

        # generate URI, e.g. 'radio://0/80/2M/247E0000'
        URI = 'radio://' + radio_specs["radio"][radio_id] + '/' + radio_specs["channel"][radio_index] + '/2M/247E0000'
        print(URI)
        # append URI's to list
        if drone_number < 10:
            uris.append(URI + "0" + str(drone_number))
            all_uris.append(URI + "0" + str(drone_number))
        else:
            uris.append(URI + str(drone_number))
            all_uris.append(URI + str(drone_number))

    uri_dict[radio_specs["radio"][radio_id]] = uris

    # generate list of parameters for the ORCA and pos_command nodes
    orca_params = dict()
    pos_comm_params = {'number_radios': number_radios}
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
                            {'number_radios': number_radios}
                           ]
                ))
    
    # launch charge manager
    launch_description.append(launch_ros.actions.Node(
                package='swarm_operation',
                executable='PadManager',
                name="PadManager",
                parameters=[
                            {'number_radios': number_radios}
                           ]
                ))
    
    # launch radio handler
    for radios in range(number_radios):
        # print(uri_dict[str(radios)])
        launch_description.append(launch_ros.actions.Node(
                    package='swarm_operation',
                    executable='RadioHandler',
                    parameters=[
                        {'uris': uri_dict[str(radios)]},
                        {'devid': radios}
                    ]))
    
    # launch GUI node
    launch_description.append(launch_ros.actions.Node(
                package='swarm_operation',
                executable='GUI',
                name="GUI",
                parameters=[
                            {'number_radios': number_radios}
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
