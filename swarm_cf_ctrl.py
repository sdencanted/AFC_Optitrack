import cflib.crtp
import pygame
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

import logging
import time

import Mocap
import DataSave
import Data_process_swarm

import math
from pyrr import quaternion
import numpy as np
import numpy.linalg as la

from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
import att_ctrl
import trajectory_generator


# robot address
# Change uris and sequences according to your setup
# channel in front, index behind 

# URI1 = 'radio://0/30/2M/E7E7E7E70B'
# URI2 = 'radio://0/30/2M/E7E7E7E707'
# URI1 = 'radio://0/30/2M/E7E7E7E703'

# URI2 = 'radio://0/30/2M/E7E7E7E70D' # problem

URI1 = 'radio://0/30/2M/E7E7E7E701'
URI2 = 'radio://0/30/2M/E7E7E7E710'

#URI2 = 'radio://0/30/2M/E7E7E7E703'
#URI3 = 'radio://0/30/2M/E7E7E7E704'


# team 1 gains (PDI)
team_1_gains= np.array([20, 10, 1]) # 20, 10, 1
# team 2 gains (PDI)
team_2_gains= np.array([20, 10, 1])


# traj
traj_chosen = 0 # 0 for hover, 1 for simple high rectangle, 2 for elevated rectangle, 3 for elevated circle, 4 for helix


uris = {
    URI1,
    URI2,
    #URI3,
}


def traj_select(traj_chosen):
    return traj_chosen


def gains_1(team_1_gains):
    kpz_1 = team_1_gains[0]
    kdz_1 = team_1_gains[1]
    kiz_1 = team_1_gains[2]
    z_gains_1 = np.array([kpz_1*1000, kdz_1*1000, kiz_1*1000])
    return z_gains_1


def gains_2(team_2_gains):
    kpz_2 = team_2_gains[0]
    kdz_2 = team_2_gains[1]
    kiz_2 = team_2_gains[2]
    z_gains_2 = np.array([kpz_2*1000, kdz_2*1000, kiz_2*1000])
    return z_gains_2


def swarm_exe(cmd_att):
    seq_args = {
        URI1: [cmd_att[0]],
        URI2: [cmd_att[1]],
        #URI3: [cmd_att[2]],
    }
    return seq_args


def init_throttle(scf, cmds):
    try:
        cf = scf.cf
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(0.1)
        cf.commander.send_setpoint(int(cmds[0]), int(cmds[1]), int(cmds[2]), int(cmds[3])) 
        print("Initialisation swarming....set ur sticks to mid and down")
        time.sleep(0.1)
    except Exception as e:
        print("Initialisation swarming error: ", e)


def arm_throttle(scf, cmds):
    try:
        cf = scf.cf
        cf.commander.send_setpoint(int(cmds[0]), int(cmds[1]), int(cmds[2]), int(cmds[3])) 
        #print('arming w thrust val....', cmds[3])
    except Exception as e:
        print("swarming error: ", e)


if __name__ == '__main__':

    data_receiver = Mocap.Udp()
    sample_rate = data_receiver.get_sample_rate()
    sample_time = 1 / sample_rate
    data_processor = Data_process_swarm.RealTimeProcessor(5, [16], 'lowpass', 'cheby2', 85, sample_rate)

    data_saver = DataSave.SaveData('Data_time',
                                   'robot_1','robot_2','ref_pos1','ref_pos2')
                                   
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    # Initialize the joysticks
    pygame.init()
    pygame.joystick.init()
    done = False
    controllerEnable = False
    pad_speed = 1
    time_last = 0
    count = 0

    # reference offset for z
    z_offset = 0.0

    # rmse terms
    rmse_num_1 = 0
    rmse_count_1 = 0
    final_rmse_1 = 0

    rmse_num_2 = 0
    rmse_count_2 = 0
    final_rmse_2 = 0
    
    with Swarm(uris, factory= CachedCfFactory(rw_cache='./cache')) as swarm:
        #swarm.reset_estimators()
        cmd_att_startup = np.array([0, 0, 0, 0]) # init setpt to 0 0 0 0
        cmd_att = np.array([cmd_att_startup,cmd_att_startup])
        seq_args = swarm_exe(cmd_att)
        swarm.parallel(init_throttle, args_dict=seq_args)


        # trajectory generator
        traj_gen = trajectory_generator.trajectory_generator()

        # team 1
        # kpz_1 = 20
        # kdz_1 = 10
        # kiz_1 = 1
        # z_gains_1 = np.array([kpz_1*1000, kdz_1*1000, kiz_1*1000])
        z_gains_1 = gains_1(team_1_gains)
        att_robot_1 = att_ctrl.att_ctrl(z_gains_1)
        
        
        # team 2
        # kpz_2 = 20
        # kdz_2 = 10
        # kiz_2 = 1
        # z_gains_2 = np.array([kpz_2*1000, kdz_2*1000, kiz_2*1000])
        z_gains_2 = gains_2(team_2_gains)
        att_robot_2 = att_ctrl.att_ctrl(z_gains_2)
        
        # team 3
        kpz_3 = 20
        kdz_3 = 10
        kiz_3 = 1
        z_gains_3 = np.array([kpz_3*1000, kdz_3*1000, kiz_3*1000])
        att_robot_3 = att_ctrl.att_ctrl(z_gains_3)
        

        time_start = time.time()
        time_end = time.time() + 6000

        while time_end > time.time():
            abs_time = time.time() - time_start
            # where hand control comes
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True

            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            lowest = 0.9
            highest = -1
            range_js = -(highest - lowest)
            range_motor = 65535
            rate = range_motor / range_js
            a0 = joystick.get_axis(0)  # x axis right hand <- ->
            a1 = joystick.get_axis(1)  # y axis right hand up down
            a2 = joystick.get_axis(2)  # thrust
            a3 = joystick.get_axis(3)

            button0 = joystick.get_button(0)
            button1 = joystick.get_button(1)

            # thrust from control pad
            conPad = int((a2 - highest) * rate)

            # joystick saturation

            if conPad < 10:
                conPad = 10
            if conPad > 65500:
                conPad = 65500

            # takeoff sign
            if conPad < 2000:
                enable = 0
            else:
                enable = 1

            # require data from Mocap
            data = data_receiver.get_data()
            # data unpack
            data_processor.data_unpack(data)
            # raw data
            raw_data = data_processor.raw_data
            # filt data
            filt_data = data_processor.get_data_filted()
            # rotation matrix
            rotm_1 = data_processor.get_rotm_1()
            rotm_2 = data_processor.get_rotm_2()
            rotm_3 = data_processor.get_rotm_3()
            # yaw
            yaw_1 = data_processor.get_heading_x1()
            yaw_2 = data_processor.get_heading_x2()
            yaw_3 = data_processor.get_heading_x3()

            # position feedback
            robot_a = [data_processor.px1, data_processor.py1, data_processor.pz1, data_processor.quat_x1, data_processor.quat_y1, data_processor.quat_z1, data_processor.quat_w1, yaw_1]
            robot_b = [data_processor.px2, data_processor.py2, data_processor.pz2, data_processor.quat_x2, data_processor.quat_y2, data_processor.quat_z2, data_processor.quat_w2, yaw_2]
            robot_c = [data_processor.px3, data_processor.py3, data_processor.pz3, data_processor.quat_x3, data_processor.quat_y3, data_processor.quat_z3, data_processor.quat_w3, yaw_3]

            # in case optitrack screws up the order somehow
            robot_1 = robot_a #2
            robot_2 = robot_b #4
            #robot_3 = robot_c #3

            # calculate velocity
            dt = time.time() - time_last  #  time difference
            time_last = time.time()

            # reference position
            traj_round = traj_select(traj_chosen)
            
            if traj_round == 0:
                ref_pos_1 = traj_gen.hover_test(-1)
                ref_pos1 = ref_pos_1[0]
                ref_pos_2 = traj_gen.hover_test(0)
                ref_pos2 = ref_pos_2[0]
                #ref_pos_3 = traj_gen.hover_test(1)
                #ref_pos3 = ref_pos_3[0]


            elif traj_round == 1:
                ref_pos_1 = traj_gen.simple_rectangle(-1, abs_time)
                ref_pos1 = ref_pos_1[0]
                ref_pos_2 = traj_gen.simple_rectangle(0, abs_time)
                ref_pos2 = ref_pos_2[0]
                #ref_pos_3 = traj_gen.low_alt_rectangle(1, abs_time)
                #ref_pos3 = ref_pos_3[0]


            elif traj_round == 2:
                ref_pos_1 = traj_gen.elevated_rectangle(-1, abs_time)
                ref_pos1 = ref_pos_1[0]
                ref_pos_2 = traj_gen.elevated_rectangle(0, abs_time)
                ref_pos2 = ref_pos_2[0]
                #ref_pos_3 = traj_gen.low_alt_rectangle(1, abs_time)
                #ref_pos3 = ref_pos_3[0]

            
            elif traj_round == 3:    
                ref_pos_1 = traj_gen.elevated_circle(-1, 0.4, count, 2)
                ref_pos1 = ref_pos_1[0]
                ref_pos_2 = traj_gen.elevated_circle(0, 0.4, count, 2)
                ref_pos2 = ref_pos_2[0]

            elif traj_round == 4:
                ref_pos_1 = traj_gen.helix(-1, 0.4, count, 5)
                ref_pos1 = ref_pos_1[0]
                ref_pos_2 = traj_gen.helix(0.5, 0.4, count, 5)
                ref_pos2 = ref_pos_2[0]


            # update positions etc.
            att_robot_1.update(robot_1, dt, ref_pos1, z_offset)
            att_robot_2.update(robot_2, dt, ref_pos2, z_offset)
            #att_robot_3.update(robot_3, dt, ref_pos3, z_offset)


            """ # control input (arming test)
            cmd_att_arm = np.array([0, 0, 0, conPad * 1]) # optitrack control [roll,  pitch ,  yawrate, thrust]
            cmd_att = np.array([cmd_att_arm, cmd_att_arm, cmd_att_arm]) """
            
            
            # control input (traj execution)
            cmd_att_1 = att_robot_1.get_angles_and_thrust(enable)
            cmd_att_2 = att_robot_2.get_angles_and_thrust(enable)
            #cmd_att_3 = att_robot_3.get_angles_and_thrust(enable)
            cmd_att = np.array([cmd_att_1, cmd_att_2])
            seq_args = swarm_exe(cmd_att)
            #print("seq_args: ", seq_args)
            swarm.parallel(arm_throttle, args_dict=seq_args)
            #print (ref_pos_2[1])

            # ref pos in array
            total_ref_pos_z = np.array([ref_pos1[2], ref_pos2[2]])

            # z feedback in array
            total_feedback_pos_z = np.array([robot_1[2], robot_2[2]])
            
            # z error in array
            total_error_z = total_ref_pos_z - total_feedback_pos_z

            count = count + 1
            if count % 10 == 0:
                
             #print(abs_time) # updating at 120 hz
                print (ref_pos_1[1]) 
                print('robot_z_position 1 2', total_feedback_pos_z)
                print('robot ref pos 1 2', total_ref_pos_z)
                print('pos_z_error', total_error_z)

            # rmse accumulation
            rmse_num_1 = rmse_num_1 + (ref_pos1[2]-robot_1[2])**2
            rmse_num_2 = rmse_num_2 + (ref_pos2[2]-robot_2[2])**2

            # save data
            data_saver.add_item(abs_time,
                                robot_1,robot_2,ref_pos1,ref_pos2
                                )

            if button0 == 1:
                """ # for hovering test
                ref_pos1[2] = 0.1
                ref_pos2[2] = 0.1
                ref_pos3[2] = 0.1
                # descend
                att_robot_1.update(robot_1, dt, ref_pos1, z_offset)
                att_robot_2.update(robot_2, dt, ref_pos2, z_offset)
                att_robot_3.update(robot_3, dt, ref_pos3, z_offset)
                cmd_att_1 = att_robot_1.get_angles_and_thrust(enable)
                cmd_att_2 = att_robot_2.get_angles_and_thrust(enable)
                cmd_att_3 = att_robot_3.get_angles_and_thrust(enable)
                cmd_att = np.array([cmd_att_1, cmd_att_2, cmd_att_3])
                seq_args = swarm_exe(cmd_att)
                swarm.parallel(arm_throttle, args_dict=seq_args) """

                # for traj or hovering test
                cmd_att_cut = np.array([0, 0, 0, 0]) # init setpt to 0 0 0 0
                cmd_att = np.array([cmd_att_cut,cmd_att_cut])
                seq_args = swarm_exe(cmd_att)
                swarm.parallel(init_throttle, args_dict=seq_args)
                final_rmse_1 = math.sqrt(rmse_num_1/count)
                final_rmse_2 = math.sqrt(rmse_num_2/count)
                print('Emergency Stopped and rmse produced for drones 1 & 2: ', final_rmse_1, final_rmse_2)
                break

# save data
#path = '/home/emmanuel/AFC_Optitrack/robot_swarm/'
#data_saver.save_data(path)

