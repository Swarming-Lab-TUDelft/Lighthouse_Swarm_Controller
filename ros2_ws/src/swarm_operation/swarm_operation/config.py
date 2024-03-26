# Configuration file 24/7 swarm Delft

#################### Example patterns ####################
MASTER_COMMANDER = "MasterCommander"  # Example patterns
COMMANDER = MASTER_COMMANDER    # Pattern to use. You can also use your custom node executable here, edit launch.py directly, or create your own launch file.

#################### GUI Config    ######################
PATTERN_GUI = False  # Simple GUI to switch between patterns for demonstrations (opens beside the main GUI)

#################### Communication ####################
NUM_CFS = 8  # Total number of drones
START_IDX_CFS = 17    # ID number of first drone

CFS_PER_RADIO = 8    # Number of drones per radio
RADIO_CHANNELS = 20, 40, 60, 80, 100    # Communication channels for each radio

#################### Bounds ####################
LH_HIGH_RISK_BOUNDS = (    # [m] High risk zone outside which the drone will land the moment it loses lighthouse positioning
    (-1.55, 1.65),          # x
    (-1.8, 1.65),         # y
    (0.0, 2.0)             # z
)
ABS_BOUNDS = (             # [m] Absolute bounds outside which the drone will emergency land
    (-1.6, 1.7),         # x
    (-1.85, 1.7),         # y
    (0.0, 2.5)             # z
)
ENABLE_LH_HIGH_RISK = False    # Enable high risk zone
ENABLE_BOUNDS = True    # Enable absolute bounds

#################### Landing ####################
LAND_H = 0.3    # [m] Height above landing pad at which it start performing the landing procedure
MIN_PAD_DIST = 0.3    # [m] Minimum distance between drones required for landing (drones will wait or land on another pad if the distance is smaller than this)
LANDING_MAX_TRIES = 5    # Number of times the drone will try to land on a pad before giving up

#################### Update rates ####################
MAIN_LOOP_UR = 10    # Rate at which the state functions are called (drone statemachine)
COMMAND_UR = 10    # Rate at which position and velocity are received and velocity commands are send during flight (max value depends on the number of drones per radio, number of radios, and system specs)
COMMAND_UR_STANDBY = 2    # Rate at which position and velocity are received while not in flight
SYSTEM_PARAM_UR = 4    # Rate at which system states are received (battery, lighthouse connection, is thumbled, ...)

#################### Miscellaneous ####################
CLIP_VEL = 1.0  # [m/s] Max collision avoidance velocity

STARTUP_TO_WAITING = False    # If True, drones will go to waiting at startup, ignoring the battery state. Else, drones will go to charging if battery is too empty.

ENABLE_YAW = False    # Enable yaw control

VELOCITY_LIMIT = 50    # [m/s] Emergency land if the drones fly faster than this limit. (Doesn't work when drones lose position tracking)

WAIT_POS_RETURN = -1.3, 0.0, 0.7    # [m] Return position before landing (will wait here until a landing pad is available)
WAIT_POS_TAKEOFF = 1.3, 0.0, 0.7    # [m] Waiting position after takeoff until a position or velocity command is received

#################### Logging ####################
LOG_LEVEL = "info"          # Log level for the swarm_operation launch file. Can be "info" or "debug"
PARAMETER_LOG_RATE = 0.5    # [s] Rate at which the drone's parameters are logged (position, velocity, battery, ...)
NUM_BASESTATIONS = 4        # Number of base stations. Only used to correctly log how many base stations are active.

#################### Collision Avoidance ####################
FULL, OPERATIONS, OFF = "full", "operations", "off"
CA_MODE = FULL
    # "full": always use collision avoidance
    # "operations": don't use collision avoidance during swarming, only at take off and returning
    # "off": fully disable collision avoidance

CA_RADIUS = 0.15    # [m] Collision sphere around drones for collision avoidance

CA_CONFIG_DIR = "Linux"    # Path to collision avoidance configuration folder. For Linux use "Linux", for WSL use "/mnt/c/Users/...../WindowsCA".