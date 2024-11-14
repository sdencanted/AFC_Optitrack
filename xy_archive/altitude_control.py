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
from scipy.optimize import fsolve


def equations(p):
    theta, phi, f = p
    R13 = math.cos(psi) * math.sin(theta) * math.cos(phi) + math.sin(psi) * math.sin(phi)
    R23 = math.sin(psi) * math.sin(theta) * math.cos(phi) - math.cos(psi) * math.sin(phi)
    R33 = math.cos(phi) * math.cos(theta)
    return [R13 * f - fx, R23 * f - fy, R33 * f - fz]


if __name__ == '__main__':

    data_receiver = Mocap.Udp()
    sample_rate = data_receiver.get_sample_rate()
    sample_time = 1 / sample_rate
    data_processor = Data_process_swarm.RealTimeProcessor(5, [16], 'lowpass', 'cheby2', 85, sample_rate)

    data_saver = DataSave.SaveData('Data_time',
                                   'data',
                                   'gains',
                                   'ref_position'
                                   )

    ###################             robot address

    #URI = 'radio://0/30/2M/E7E7E7E703'
    #URI = 'radio://0/20/2M/E7E7E7E702'
    #URI = 'radio://0/40/2M/E7E7E7E706'
    URI = 'radio://0/60/2M/E7E7E7E709'

    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

  # Initialize the joysticks
    pygame.init()
    pygame.joystick.init()
    done = False
    controllerEnable = False
    pad_speed = 1

    # control gains
    kpx = 20_000
    kpy = 20_000

    kdx = 15_000
    kdy = 15_000

    kix = 0
    kiy = 0

    #                                 attitude gains ########################


    # other parameters initialization
    count = 0

    psi = 0
    fx = 0
    fy = 0
    fz = 0

    px_last = 0
    py_last = 0
    pz_last = 0
    time_last = -100
    I_term_z = 0
    b1_last = 1
    time_stamp_b1 = 0
    competition_start = 0

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
            # where hand control comes
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True

            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            lowest = 0.787
            highest = -0.89
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
            robot_1 = [data_processor.px1, data_processor.py1, data_processor.pz1, yaw_1]
            robot_2 = [data_processor.px2, data_processor.py2, data_processor.pz2, yaw_2]
            robot_3 = [data_processor.px3, data_processor.py3, data_processor.pz3, yaw_3]

            ####################################################    select robot id in Optitrack system
            robot = robot_1

            #   altitude control gains  -- default
            kpz = 20_000  # proportional gain
            kiz = 10_000  # integral gain
            kdz = 15_000  # derivative gain
            robot_mass = 30_000  # constant value

            #                                  gains need to tune
            #                                  range of kpz: (10,80)
            #                                  range of kiz: (10, 200)
            #                                  range of kdz: (10, 200)
            gain_kp_alt = 26.3
            gain_ki_alt = 1.5
            gain_kd_alt = 8.2

            # remapping
            kpz_test = gain_kp_alt * 1000  # proportional gain
            kiz_test = gain_ki_alt * 1500  # integral gain
            kdz_test = gain_kd_alt * 1000  # derivative gain

            # kpz = 25_000  # proportional gain
            # kiz = 10_000  # integral gain
            # kdz = 15_000  # derivative gain

            # desired trajectory - hovering
            # px_s = -1
            # py_s = 0.5
            # pz_s = 0.2

            if abs_time < 5:
                px_s = 1
                py_s = -0.4
                pz_s = 0.2
            elif 5 <= abs_time < 12:
                px_s = 1
                py_s = -0.4
                pz_s = 0.5
                kpz = kpz_test
                kiz = kiz_test
                kdz = kdz_test

            elif 12 <= abs_time < 19:
                px_s = 1
                py_s = 0.4
                pz_s = 1.2
                kpz = kpz_test
                kiz = kiz_test
                kdz = kdz_test

            elif 19 <= abs_time < 26:
                px_s = 1
                py_s = -0.4
                pz_s = 0.5
                kpz = kpz_test
                kiz = kiz_test
                kdz = kdz_test
            elif 26 <= abs_time < 33:
                px_s = 1
                py_s = 0.4
                pz_s = 1.2
                kpz = kpz_test
                kiz = kiz_test
                kdz = kdz_test
            elif 33 <= abs_time < 40:
                px_s = 1
                py_s = -0.4
                pz_s = 0.5
                kpz = kpz_test
                kiz = kiz_test
                kdz = kdz_test
            else:
                px_s = 1
                py_s = -0.4
                pz_s = 0.2

            # landing sign
            if button1 == 0:
                px_s = 1
                py_s = -0.4
                pz_s = 0.1

            # update position
            px_m = robot[0]
            py_m = robot[1]
            pz_m = robot[2]

            # calculate velocity
            dt = time.time() - time_last   #  time difference
            time_last = time.time()

            vx = (px_m - px_last) / dt
            vy = (py_m - py_last) / dt
            vz = (pz_m - pz_last) / dt

            px_last = px_m
            py_last = py_m
            pz_last = pz_m

            px_err = px_s - px_m
            py_err = py_s - py_m
            pz_err = pz_s - pz_m   # calculate position error

            # pid controller
            fx_d = kpx * px_err - kdx * vx
            fy_d = kpy * py_err - kdy * vy

            # integration term for z position
            I_term_z = enable * kiz * pz_err * dt / 2 + I_term_z

            #                                                          desired fz calculation
            fz_d = kpz * pz_err - kdz * vz + I_term_z + robot_mass

            fx = fx_d
            fy = fy_d
            fz = fz_d * enable + 10_000

            psi = robot[3]

            des_pitch, des_roll, des_thrust = fsolve(equations, [0, 0, 30000])

            des_roll = int(des_roll * 180 / math.pi)
            des_pitch = int(des_pitch * 180 / math.pi)

            # output saturation
            if des_roll > 40:
                des_roll = 40
            if des_roll < -40:
                des_roll = -40
            if des_pitch > 40:
                des_pitch = 40
            if des_pitch < -40:
                des_pitch = -40
            if des_thrust > 60_000:
                des_thrust = 60_000
            if des_thrust < 10:
                des_thrust = 10

            thrust = int(des_thrust)  # for optitrack control

            roll_set = -a0 * 20 / 0.835  # degree
            pitch_set = -a1 * 20 / 0.835  # degree
            yaw_rate_set = -a3 * 20 / 0.835  # degree/s

            cf.commander.send_setpoint(des_roll, des_pitch, 0, thrust * 1 + conPad * 0)    # optitrack control [roll,  pitch ,  yawrate, thrust]

            count = count + 1
            print('robot_position', robot)

            # save data
            gains = [kpz, kiz, kdz, robot_mass]
            ref_position = [px_s, py_s, pz_s]
            data_saver.add_item(abs_time,
                                robot,
                                gains,
                                ref_position
                                )

            if button0 == 1:
                cf.commander.send_setpoint(0, 0, 0, 10)
                print('Emergency Stopped')
                break

        # save data
        path = '/Users/airlab/PycharmProjects/AFC/data/robot_1_'
        data_saver.save_data(path)

