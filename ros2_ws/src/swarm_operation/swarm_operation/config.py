# Configuration file 24/7 swarm Delft

#################### Communication ####################
NUM_CFS = 10    # Total number of drones
START_IDX_CFS = 14    # ID number of first drone

CFS_PER_RADIO = 8    # Number of drones per radio
RADIO_CHANNELS = 20, 40, 60, 80, 100    # Communication channels for each radio

#################### Bounds ####################
LH_HIGH_RISK_BOUNDS = (    # [m] High risk zone outside which the drone will land the moment it loses lighthouse positioning
    (-1.53, 1.5),          # x
    (-1.69, 1.82),         # y
    (0.0, 2.0)             # z
)
ABS_BOUNDS = (             # [m] Absolute bounds outside which the drone will emergency land
    (-1.78, 1.75),         # x
    (-1.84, 1.97),         # y
    (0.0, 2.5)             # z
)
ENABLE_LH_HIGH_RISK = False    # Enable high risk zone
ENABLE_BOUNDS = True    # Enable absolute bounds

#################### Landing ####################
LAND_H = 0.3    # [m] Height above landing pad at which it start performing the landing procedure
MIN_PAD_DIST = 0.3    # [m] Minimum distance between drones required for landing (drones will wait or land on another pad if the distance is smaller than this)

#################### Update rates ####################
MAIN_LOOP_UR = 10    # Rate at which the state functions are called (drone statemachine)
COMMAND_UR = 10    # Rate at which position and velocity are received and velocity commands are send during flight (max value depends on the number of drones per radio, number of radios, and system specs)
COMMAND_UR_STANDBY = 2    # Rate at which position and velocity are received while not in flight
SYSTEM_PARAM_UR = 4    # Rate at which system states are received (battery, lighthouse connection, is thumbled, ...)

#################### Miscellaneous ####################
STARTUP_TO_WAITING = True    # If True, drones will go to waiting at startup, ignoring the battery state. Else, drones will go to charging if battery is too empty.

ENABLE_YAW = False    # Enable yaw control

VELOCITY_LIMIT = 50    # [m/s] Emergency land if the drones fly faster than this limit. (Doesn't work when drones lose position tracking)

CA_RADIUS = 0.15    # [m] Collision sphere around drones for collision avoidance
