import cflib.crtp
import pygame
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

import logging
import time

import DataSave
import Data_process_vrpn

import math
from pyrr import quaternion
import numpy as np
import numpy.linalg as la

from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
import att_ctrl
import trajectory_generator

import vrpn
import threading
import yaml
# seems like VRPN sends each rigid body as a separate tracker, so there is no point trying to group them in 1 callback
class VrpnTracker():
#create a vrpn tracker object
    def handle_position(self, _, data):
        self.lock.acquire()

        #1st axis is front aka +ve x
        #2nd axis is left aka -ve z
        #3rd axis is up aka +ve y

        #crazyflie x is forward
        #crazyflie y is left
        #crazyflie z is up

        #assign xyz based on crazyflie frame


        self.position[0] = data['position'][0]  # x
        self.position[1] = data['position'][1]  # y
        self.position[2] = data['position'][2]  # z
        self.quaternion[0] = data['quaternion'][0]  # qx
        self.quaternion[1] = data['quaternion'][1]  # qy
        self.quaternion[2] = data['quaternion'][2]  # qz
        self.quaternion[3] = data['quaternion'][3]  # qw
        self.lock.release()
    def get_data(self):
        if self.quaternion[3]==-2:
            print("waiting for first data...")
            while self.quaternion[3]==-2:
                time.sleep(0.1)
            print("first data received,", self.position)
        self.lock.acquire()
        #deep copy positions and quaternions
        position = self.position[:]
        quaternion = self.quaternion[:]
        self.lock.release()
        return position, quaternion
    def run(self):
        while True:
            self.tracker.mainloop()
    def __init__(self,name="drone_3",address="192.168.65.4"):
        self.name=name
        # data={"x":0,"y":0,"z":0,"qx":0,"qy":0,"qz":0,"qw":-2}
        # create a multiprocessing shared value for data
        # self.data = self.manager.dict(data)
        self.position =[0.0,0.0,0.0]
        self.quaternion = [0.0,0.0,0.0,-2.0]


        self.tracker = vrpn.receiver.Tracker(name+"@"+address)
        self.tracker.register_change_handler(None,self.handle_position,"position")
        # self.tracker.register_change_handler(self,lambda tracker,data:tracker.handle_position(None,data),"position")

        # create a lock on self.data
        self.lock = threading.Lock()

        # create a multiprocessing process to run the vrpn tracker
        self.process = threading.Thread(target=self.run)
        self.process.daemon = True
        self.process.start()


if __name__ == '__main__':

    #import yaml config
    config=yaml.safe_load(open('swarm_uris_gains.yaml'))
    skip_joystick = config['skip_joystick']



    traj_chosen = config["traj_chosen"] # 0 for hover, 1 for low rectangle, 2 for simple high rectangle, 3 for elevated circle, 4 for helix

    global_x_offset = config["global_x_offset"]
    global_y_offset = config["global_y_offset"]
    x_range=config["x_range"]
    y_range=config["y_range"]
    z_range=config["z_range"]
    vrpn_server_ip= config["server_ips"][config["chosen_server"]]
    sample_rate = config["optitrack_rate"]
    enable_flying = config["enable_flying"]
    drones= [VrpnTracker(drone['rigidbody_name'], vrpn_server_ip) for drone in config["drones"]]
    while True:
        time.sleep(0.1)
        for drone in drones:
            print(drone.get_data() )