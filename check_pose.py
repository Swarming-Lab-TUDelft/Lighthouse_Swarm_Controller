import logging
import time
import os
import numpy as np

from threading import Thread

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

"""
This script is used to check the Crazyflies are within bounds w/ a stable pose, using one radio at a time.
Edit the URI_IDX and CHANNEL variables to your needs.
"""


URI_IDX = 7, 9
CHANNEL = 60
CHECK_LEN = 200
DPOSE_THRESH = 0.1

uri_string = f'radio://0/{CHANNEL}/2M/247E'

def check_pose(URI):
    try:
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            lg_pose = LogConfig(name='StateEstimate', period_in_ms=10)
            lg_pose.add_variable('stateEstimate.x', 'float')
            lg_pose.add_variable('stateEstimate.y', 'float')
            lg_pose.add_variable('stateEstimate.z', 'float')
            with SyncLogger(scf, lg) as logger:
                
                check = np.zeros((CHECK_LEN, 3)) 
                for log_idx, log_entry in enumerate(logger):
                    if log_idx == check.shape[0]:
                        break
                    timestamp = log_entry[0]
                    data = log_entry[1]
                    logconf_name = log_entry[2]

                    # populate check array
                    check[log_idx, 0] = data['stateEstimate.x']
                    check[log_idx, 1] = data['stateEstimate.y']
                    check[log_idx, 2] = data['stateEstimate.z']
               
                check_min, check_max = np.min(check, axis=0), np.max(check, axis=0)
                dpose = np.sqrt(np.sum(np.power(check_min - check_max, 2.0)))
                print(f"{URI} : {dpose}")
                if dpose > DPOSE_THRESH:
                    print(f"Pose unstable for Crazyflie with URI: {URI}")

                # wait for 5 seconds
                time.sleep(5)

    except:
        print("Error in connecting to Crazyflie with URI: " + URI)
        pass

if __name__ == '__main__':

    cflib.crtp.init_drivers()

    # initialise logger -> TODO change channels
    lg_pose = LogConfig(name='StateEstimate', period_in_ms=10)
    lg_pose.add_variable('stateEstimate.x', 'float')
    lg_pose.add_variable('stateEstimate.y', 'float')
    lg_pose.add_variable('stateEstimate.z', 'float')

    # list of threads
    threads = []
    for i in range(URI_IDX[0], URI_IDX[1]+1):

        uri = uri_string + '0'*(6-len(str(i))) + str(i)

        # start a new thread
        t = Thread(target=check_pose, args=(uri,))
        t.start()

        # add thread to list
        threads.append(t)
    
    # wait for all threads to finish
    for t in threads:
        t.join()

            
