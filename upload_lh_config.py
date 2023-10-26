import logging
import time
import os

from threading import Thread

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.localization import LighthouseConfigWriter

"""
This script is used to upload the configuration file to the Crazyflies using one radio at a time.
Edit the URI_IDX and CHANNEL variables to your needs.
"""


URI_IDX = 17, 24     # (start, end) index of URI's to be used (including start and end)
CHANNEL = 60        # radio channel to be used



uri_string = f'radio://0/{CHANNEL}/2M/247E'

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

    for i in range(URI_IDX[0], URI_IDX[1]+1):

        uri = uri_string + '0'*(6-len(str(i))) + str(i)

        # start a new thread
        t = Thread(target=upload_settings, args=(uri,))
        t.start()

        # add thread to list
        threads.append(t)
    
    # wait for all threads to finish
    for t in threads:
        t.join()


            