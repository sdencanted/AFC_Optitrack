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
import yaml
# seems like VRPN sends each rigid body as a separate tracker, so there is no point trying to group them in 1 callback
class VrpnTracker():
#create a vrpn tracker object
    def handle_position(self, _, data):
        # self.lock.acquire()

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
        # self.lock.release()
    def get_data(self):
        self.quaternion[3]=-2
        while self.quaternion[3]==-2:
            self.tracker.mainloop()
        # self.lock.acquire()
        #deep copy positions and quaternions
        position = self.position[:]
        quaternion = self.quaternion[:]
        # self.lock.release()
        return position, quaternion
    # def run(self):
    #     while True:
    #         self.tracker.mainloop()
    def __init__(self,name="drone_3",address="192.168.65.4"):
        self.name=name
        # data={"x":0,"y":0,"z":0,"qx":0,"qy":0,"qz":0,"qw":-2}
        # create a multiprocessing shared value for data
        # self.data = self.manager.dict(data)
        self.position =[0.0,0.0,0.0]
        self.quaternion = [0.0,0.0,0.0,-2.0]


        self.tracker = vrpn.receiver.Tracker(name+"@"+address)
        self.tracker.register_change_handler(None,self.handle_position,"position")



class Drone:
    def __init__(self, args_dict,address="192.168.65.4",x_offset=0.0,y_offset=0.0,x_range=(-100,100),y_range=(-100,100),z_range=(-100,100),sample_rate=100):
        self.rigidbody_name = args_dict['rigidbody_name']
        self.uri = args_dict['uri']
        self.tracker= VrpnTracker(self.rigidbody_name, address)
        
        # get initial position for offset
        self.local_offset=np.array([0,0,0],dtype=np.float32)
        position,_ = self.tracker.get_data()
        self.local_offset[0] = position[0]+ x_offset
        self.local_offset[1] = position[1]+ y_offset
        self.local_offset[2] = position[2]
        print("local offset for ", self.rigidbody_name, ":", self.local_offset)

        # it is scaled like that. idk why.
        self.gains = np.array(args_dict['gains'])*1000 
        # print(self.gains)
        # exit()
        self.att_ctrl=att_ctrl.att_ctrl(self.gains)

        self.data_saver = DataSave.SaveData('Data_time', self.rigidbody_name,'ref_position')
        self.rmse=np.array([0,0,0],dtype=np.float32) # x,y,z

        self.data_processor = Data_process_vrpn.RealTimeProcessor(5, 16, 'lowpass', 'cheby2', 85, sample_rate)
        self.state=None
        self.error = np.array([0,0,0],dtype=np.float32)
        self.rmse_x_num = 0
        self.rmse_y_num = 0
        self.rmse_z_num = 0
        self.ref_pos_min = np.array([x_range[0], y_range[0], z_range[0]],dtype=np.float32)
        self.ref_pos_max = np.array([x_range[1], y_range[1], z_range[1]],dtype=np.float32)
        print("drone ",self.rigidbody_name," setup done")
    def apply_position_offsets(self, cmd_att):
        new_cmd_att=cmd_att.copy()
        # apply local offset to the command
        new_cmd_att[0] = cmd_att[0] + self.local_offset[0]
        new_cmd_att[1] = cmd_att[1] + self.local_offset[1]
        # cmd_att[2] = cmd_att[2] + self.local_offset[2]
        # return new_cmd_att
        return np.clip(new_cmd_att,self.ref_pos_min,self.ref_pos_max)
    def generate_cmd(self,ref_pos,dt,enable_flying=True):
        # get the current position from the vrpn tracker
        position,quaternion = self.tracker.get_data()
        self.data_processor.data_unpack_vrpn(position, quaternion) # unpack the data

        # position feedback
        self.state= self.data_processor.get_state()
        ref_pos_modified = self.apply_position_offsets(ref_pos)
        self.data_saver.add_item(abs_time,
                                self.state,ref_pos_modified
                                )
        # update positions etc.
        # calculate the attitude angles and thrust
        self.att_ctrl.update(self.state, dt, ref_pos_modified, 0)
        # control input (traj execution)
        cmd_att = self.att_ctrl.get_angles_and_thrust(True)
        # save data
        
        self.error[0] = ref_pos_modified[0] - self.state[0]
        self.error[1] = ref_pos_modified[1] - self.state[1]
        self.error[2] = ref_pos_modified[2] - self.state[2]
        

        # rmse accumulation
        self.rmse_x_num += (ref_pos_modified[0]-self.state[0])**2
        self.rmse_y_num += (ref_pos_modified[1]-self.state[1])**2
        self.rmse_z_num += (ref_pos_modified[2]-self.state[2])**2
        return [cmd_att,enable_flying]
    def show_rmse(self):
        final_x_rmse = math.sqrt(self.rmse_x_num/count)
        final_y_rmse = math.sqrt(self.rmse_y_num/count)
        final_z_rmse = math.sqrt(self.rmse_z_num/count)
        print('Drone:', self.rigidbody_name, 'rmse produced: ', final_x_rmse,final_y_rmse,final_z_rmse)
    



# cmds: [roll, pitch, yawrate, thrust]
def init_throttle(scf, cmds, enable=True):
    if enable:
        try:
            cf = scf.cf
            cf.param.set_value('kalman.resetEstimation', '1')
            time.sleep(0.1)
            cf.param.set_value('kalman.resetEstimation', '0')
            time.sleep(0.1)
            cf.commander.send_setpoint(int(cmds[0]), int(cmds[1]), int(cmds[2]), int(cmds[3])) 
            time.sleep(0.1)
        except Exception as e:
            print("Initialisation swarming error: ", e)


def arm_throttle(scf, cmds, enable=True):
    if enable:
        try:
            cf = scf.cf
            # print(cmds)
            cf.commander.send_setpoint(int(cmds[0]), int(cmds[1]), int(cmds[2]), int(cmds[3])) 
            #print('arming w thrust val....', cmds[3])
        except Exception as e:
            print("swarming error: ", e)

def interpret_joystick_switches(joystick):
    # axis 0 roll
    roll = joystick.get_axis(0)
    # axis 1 pitch
    pitch = joystick.get_axis(1)
    # axis 2 thrust
    thrust = joystick.get_axis(2)
    # axis 3 yaw
    yaw = joystick.get_axis(3)
    # axis 4 left bigger deduction than right
    left_right_stick=joystick.get_axis(4)
    if left_right_stick<-0.7:
        left_stick=-1 #up
        right_stick=-1 #up
    elif left_right_stick<-0.404:
        left_stick=-1 #up
        right_stick=0 #mid
    elif left_right_stick<-0.3:
        left_stick=0 #mid
        right_stick=-1 #up
    elif left_right_stick<-0.1:
        left_stick=-1 #up
        right_stick=1 #down
    elif left_right_stick<0.05:
        left_stick=0 #mid
        right_stick=0 #mid
    elif left_right_stick<0.2:
        left_stick=1 #down
        right_stick=-1 #up
    elif left_right_stick<0.4:
        left_stick=0 #mid
        right_stick=1 #down
    elif left_right_stick<0.6:
        left_stick=1 #down
        right_stick=0 #mid
    elif left_right_stick<0.9:
        left_stick=1 #down
        right_stick=1 #down
    else:
        print(left_right_stick)
        left_stick=-1 #up
        right_stick=-1 #up
    return roll, pitch, thrust, yaw, left_stick, right_stick

        

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
    drones= [Drone(drone, vrpn_server_ip,global_x_offset,global_y_offset,x_range,y_range,z_range,sample_rate) for drone in config["drones"]]  
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

  # Initialize the joysticks
    if not skip_joystick:
        pygame.init()
        pygame.joystick.init()
    done = False
    controllerEnable = False
    pad_speed = 1
    time_last = 0
    count = 0

    # rmse terms
    uris= {drone.uri for drone in drones}
    
    no_thrust_drone_command_dict={drone.uri:[np.array([0, 0, 0, 0]),enable_flying] for drone in drones}
    print("enter swarm tab")
    with Swarm(uris, factory= CachedCfFactory(rw_cache='./cache')) as swarm:
        print("in swarm tab")
        # trajectory generator
        for drone in drones:
            drone.tracker.tracker.mainloop()
        traj_gen = trajectory_generator.trajectory_generator()

               

        if not skip_joystick:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()

            print("reset middle switch upwards")
            while True:

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        done = True
                # axis 5 middle
                arm=joystick.get_axis(5) #middle switch
                time.sleep(0.1)
                if arm<0:
                    break
            print("waiting for arm switch (middle) to go down")
            while True:

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        done = True
                # axis 5 middle
                arm=joystick.get_axis(5) #middle switch
                time.sleep(0.1)
                if arm>0:
                    break
        print("armed, starting in 3 seconds")
        time.sleep(3)
        swarm.parallel(init_throttle, args_dict=no_thrust_drone_command_dict)
        print("started")
        time_start = time.time()
        time_end = time.time() + 6000
        
        while time_end > time.time():
            abs_time = time.time() - time_start
            # where hand control comes
            if not skip_joystick:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        done = True

            lowest = -0.835
            highest = 0.811           
            range_js = (highest - lowest)
            range_motor = 65535
            rate = range_motor / range_js
            if not skip_joystick:
                roll,pitch,thrust,yaw,left_stick,right_stick=interpret_joystick_switches(joystick)
                
                # thrust from control pad
                conPad = int((thrust - highest) * rate)

            # calculate velocity
            dt = time.time() - time_last  #  time difference
            time_last = time.time()

            # reference position
            traj_round = traj_chosen
            
            if traj_round == 0:
                ref_pos,msg = traj_gen.hover_test(0)
            elif traj_round == 1:
                ref_pos,msg = traj_gen.simple_rectangle(0, abs_time)
            elif traj_round == 2:
                ref_pos,msg = traj_gen.elevated_rectangle(0, abs_time)
            elif traj_round == 3:    
                ref_pos,msg = traj_gen.elevated_circle(0, 0.4, count, 2)
            elif traj_round == 4:
                ref_pos,msg = traj_gen.helix(0, 0.4, count, 5)
            
            drone_commands={drone.uri:drone.generate_cmd(ref_pos,dt,enable_flying) for drone in drones}
            swarm.parallel(arm_throttle, args_dict=drone_commands)

            count = count + 1
            if count % 100 == 0:
                print('dt: ',dt)
                print("ref pos:", ref_pos)
                for drone in drones:
                    print('robot_position', drone.state)
                    command=drone_commands[drone.uri][0]
                    print('roll forward:', command[1], 'pitch right:', command[0], 'thrust:', command[3])
                    print('x pos_error', drone.error[0])
                    print('y pos_error', drone.error[1])
                    print('z pos_error', drone.error[2])

            
            if (not skip_joystick and right_stick > -1) or (traj_round==1 and abs_time>34) or (traj_round==0 and abs_time>10):
                
                """ # for hovering test
                ref_pos[2] = 0.15
                # descend
                att_robot_1.update(robot, dt, ref_pos, 0)
                cmd_att_1 = att_robot_1.get_angles_and_thrust(True)
                cmd_att = np.array([cmd_att_1])
                cmd_dict = generate_cmd_dict(cmd_att,uris)
                swarm.parallel(arm_throttle, args_dict=(cmd_dict,enable_flying)) """

                # for traj 
                swarm.parallel(init_throttle, args_dict=no_thrust_drone_command_dict)
                for drone in drones:
                    drone.show_rmse()
                break

# save data
#path = '/home/emmanuel/AFC_Optitrack/robot_solo/'
#data_saver.save_data(path)

