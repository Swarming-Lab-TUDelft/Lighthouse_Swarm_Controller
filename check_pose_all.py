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

from ros2_ws.src.swarm_operation.swarm_operation.config import RADIO_CHANNELS, START_IDX_CFS, NUM_CFS, CFS_PER_RADIO

"""
This script is used to check the Crazyflies are within bounds w/ a stable pose, using one radio at a time.
Edit the URI_IDX and CHANNEL variables to your needs.
"""

radios = list(RADIO_CHANNELS)# [20, 40, 60, 80]
start_idx, end_idx = START_IDX_CFS, START_IDX_CFS + NUM_CFS
cfs_per_radio = CFS_PER_RADIO

# URI_IDX = 1, 1
# CHANNEL = 20
CHECK_LEN = 200
DPOSE_THRESH = 0.1

# uri_string = f'radio://0/{CHANNEL}/2M/247E'

class PoseChecker:
    def __init__(self):
        self.check = np.zeros((CHECK_LEN, 3))

    def check_pose(self, URI, lg):
        print(f"Beginning of check_pose {uri}")
        try:
            with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
                with SyncLogger(scf, lg) as logger:
                
                    print(f"starting {URI}") 
                    self.check = np.zeros((CHECK_LEN, 3)) 
                    for log_idx, log_entry in enumerate(logger):
                        if log_idx >= self.check.shape[0]:
                            break
                        timestamp = log_entry[0]
                        data = log_entry[1]
                        logconf_name = log_entry[2]

                        # populate check array
                        self.check[log_idx, 0] = data['stateEstimate.x']
                        self.check[log_idx, 1] = data['stateEstimate.y']
                        self.check[log_idx, 2] = data['stateEstimate.z']
                        
                        print(f"{URI} : {log_idx}")
                        print(f"{URI} : {self.check[log_idx]}")
                        
                    check_min, check_max = np.min(self.check, axis=0), np.max(self.check, axis=0)
                    dpose = np.sqrt(np.sum(np.power(check_min - check_max, 2.0)))
                    # print(check_min, check_max)
                    print(f"{URI} : ", dpose)
                    # print(f"{URI} : {np.var(check, axis=0)}")
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
    objs = []
    threads = []
    for i in range(start_idx, end_idx):
        radio_idx = (i - 1) // cfs_per_radio if cfs_per_radio > 1 else i // cfs_per_radio
        print(radio_idx)
        if radio_idx >= len(radios):
            e = "Too many Crazyflies or too few radio channels provided. Unable to access Crazyfly {i}."
        uri_string = f'radio://0/{radios[radio_idx]}/2M/247E'

        uri = uri_string + '0'*(6-len(str(i))) + str(i)

        print("Checking uri : ", uri)

        # start a new thread
        obj = PoseChecker()
        t = Thread(target=obj.check_pose, args=(uri, lg_pose,))
        t.start()
    
        # add thread to list
        threads.append(t)
    
    # wait for all threads to finish
    for t in threads:
        t.join()

            
