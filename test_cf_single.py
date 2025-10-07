import cflib.crtp
import numpy as np
import time
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
URI1 = 'radio://0/30/2M/E7E7E7E708'

uris = {
    URI1,
}

def swarm_exe(cmd_att):
    seq_args = {
        URI1: [cmd_att[0]],
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

cflib.crtp.init_drivers()
with Swarm(uris, factory= CachedCfFactory(rw_cache='./cache')) as swarm:
    #swarm.reset_estimators()
    cmd_att_startup = np.array([0, 0, 0, 0]) # init setpt to 0 0 0 0
    cmd_att = np.array([cmd_att_startup])
    seq_args = swarm_exe(cmd_att)
    time.sleep(9)
    swarm.parallel(init_throttle, args_dict=seq_args)
    cmd_att_1 = np.array([-10,-10,  0, 60000])
    cmd_att = np.array([cmd_att_1])
    seq_args = swarm_exe(cmd_att)
    swarm.parallel(arm_throttle, args_dict=seq_args)
