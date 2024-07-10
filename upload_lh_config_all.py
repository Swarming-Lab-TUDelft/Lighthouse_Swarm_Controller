import logging
import time
import os

from threading import Thread

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.localization import LighthouseConfigWriter

from ros2_ws.src.swarm_operation.swarm_operation.config import RADIO_CHANNELS, START_IDX_CFS, NUM_CFS, CFS_PER_RADIO

"""
This script is used to upload the configuration file to the Crazyflies using one radio at a time.
Edit the URI_IDX and CHANNEL variables to your needs.
"""


# URI_IDX = 7, 9
# CHANNEL = 60

radios = list(RADIO_CHANNELS)# [20, 40, 60, 80]
start_idx, end_idx = START_IDX_CFS, START_IDX_CFS + NUM_CFS
cfs_per_radio = CFS_PER_RADIO

# uri_string = f'radio://0/{CHANNEL}/2M/247E'

logging.basicConfig(level=logging.DEBUG)

DEFAULT_HEIGHT = 0.5
BOX_LIMIT = 0.5

def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(3)
        mc.stop()

# define callback for when file has been written to cf
def write_callback(succes):
    if succes:
        print("Configuration written to Crazyflie")
    else:
        print("Failed to write configuration to Crazyflie")

def upload_settings(URI):

    try:
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

            # reset estimators
            # scf.cf.param.set_value('kalman.resetEstimation', '1')

            # create configwriter
            lighthouse_config_writer = LighthouseConfigWriter(scf.cf)

            # define path of config file
            config_file = os.path.join(os.path.dirname(__file__), 'configuration.yaml')

            # load config file
            lighthouse_config_writer.write_and_store_config_from_file(write_callback, config_file)

            # wait for 5 seconds
            time.sleep(10)

            # try to fly
            # take_off_simple(scf)
    except:
        print("Error in connecting to Crazyflie with URI: " + URI)
        pass

if __name__ == '__main__':

    cflib.crtp.init_drivers()

    # list of threads
    threads = []

    for i in range(start_idx, end_idx):
        radio_idx = (i - 1) // cfs_per_radio if cfs_per_radio > 1 else i // cfs_per_radio
        print(radio_idx)
        if radio_idx >= len(radios):
            e = "Too many Crazyflies or too few radio channels provided. Unable to access Crazyfly {i}."
        uri_string = f'radio://0/{radios[radio_idx]}/2M/247E'
        uri = uri_string + '0'*(6-len(str(i))) + str(i)

        # start a new thread
        t = Thread(target=upload_settings, args=(uri,))
        t.start()

        # add thread to list
        threads.append(t)
    
    # wait for all threads to finish
    for t in threads:
        t.join()


            
