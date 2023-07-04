import logging
import sys
import time
import os
from threading import Event

#import multithreading
from threading import Thread


from cflib.crazyflie.syncLogger import SyncLogger

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.localization import LighthouseConfigWriter

uri_string = 'radio://0/60/2M/247E000001'
uri_string_90 = 'radio://0/20/2M/247E0000'

# URI = uri_helper.uri_from_env(default=uri_string_90)

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

            print("starting to fly")
            # try to fly
            take_off_simple(scf)
    except:
        print("Error in connecting to Crazyflie with URI: " + URI)
        pass

if __name__ == '__main__':

    cflib.crtp.init_drivers()

    # list of threads
    threads = []

    for i in range(8,9):
        
        if i < 10:
            URI = uri_string_90 + '0' + str(i)
        else:
            URI = uri_string_90 + str(i)

        # start a new thread
        t = Thread(target=upload_settings, args=(URI,))
        t.start()

        # add thread to list
        threads.append(t)
    
    # wait for all threads to finish
    for t in threads:
        t.join()


            