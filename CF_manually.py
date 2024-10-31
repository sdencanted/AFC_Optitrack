import cflib.crtp
import pygame
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

import logging
import time


if __name__ == '__main__':

    # robot address

    #URI = 'radio://0/80/2M/E7E7E7E7E7'  # cf_2.1
    URI = 'radio://0/114/2M/E7E7E7E70B'  # bolt

    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

  # Initialize the joysticks
    pygame.init()
    pygame.joystick.init()
    done = False
    controllerEnable = False
    pad_speed = 1

    count = 0

    with SyncCrazyflie(URI, cf = Crazyflie(rw_cache = './cache')) as scf:
        cf = scf.cf
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)
        cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)

        time_start = time.time()
        time_end = time.time() + 6000

        while time_end > time.time():
            abs_time = time.time() - time_start
            #where hand control comes
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True

            joystick = pygame.joystick.Joystick(0)
            joystick.init()

            a0 = joystick.get_axis(0)  # x axis right hand <- ->
            a1 = -joystick.get_axis(1)  # y axis right hand up down
            a2 = joystick.get_axis(2)  # thrust
            a3 = -joystick.get_axis(3)

            L1 = joystick.get_button(9) # default = 0
            R1 = joystick.get_button(10)  # default = 0

            vx = a3
            vy = a2
            vz = a1

            if L1 == 1:
                cf.commander.send_velocity_world_setpoint(vx, vy, vz, 0)  # vx vy vz yawrate
                #cf.commander.send_hover_setpoint(vx, vy, 0, vz) # vx, vy, yaw_rate, zdistance
            else:
                cf.commander.send_setpoint(0, 0, 0, 0)
                print("press and hold L1 to actuate the communication")

            count = count + 1

            if count % 200 == 0:
                print('-----', 'time: ', abs_time, '-----')
                print('vx', vx)
                print('vy', vy)
                print('vz', vz)

            if R1 == 1:
                cf.commander.send_setpoint(0, 0, 0, 0)
                print('Emergency Stopped')
                break
