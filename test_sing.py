
import cflib.crtp
import numpy as np
import time
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
# create enum for converting notes to frequency


def duplicate_command(uris,cmd_att):
    # seq_args = {
    #     URI1: [cmd_att],
    # }
    seq_args={ uri:[cmd_att] for _,uri in enumerate(uris)}
    # print(seq_args)
    return seq_args
# def sing(scf,freq:list=[100,100,100,100],duration_ms:int=500):
def sing(scf,args):
    # print(args)

    freq=args.get('freq',[100,100,100,100])
    freq=[freq_val for freq_val in freq if freq_val>261]
    duration_ms=args.get('duration_ms',500)
    try:
        # print(freq)
        # print(duration_ms)
        cf = scf.cf
        if len(freq)<1:
            # print("no freq provided for singing")
            return
        elif len(freq)==1:
            freq=freq*4
        elif len(freq)==2:
            freq=freq*2
        elif len(freq)==3:
            freq=freq+[freq[2]]
        elif len(freq)>4:
            freq=freq[:4]
        cf.param.set_value('sing.f0',str(freq[0]))
        cf.param.set_value('sing.f1',str(freq[1]))
        cf.param.set_value('sing.f2',str(freq[2]))
        cf.param.set_value('sing.f3',str(freq[3]))
        cf.param.set_value('sing.d',str(duration_ms))
        cf.param.set_value('sing.s','1')
        while cf.param.param_updater.request_queue.qsize() > 0:
            time.sleep(0.01)
        # while cf.param.get_value('sing.s') =='1':
        #     time.sleep(0.01)
    except Exception as e:
        print("singing error: ", e)
def wait_for_sing(scf):
    cf=scf.cf
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

def sing_swarm(uris,swarm,freq,duration_ms):
    # print("singing freq:",freq," for ",duration_ms," ms")
    sing_arg=duplicate_command(uris,{'freq':freq,'duration_ms':duration_ms})
    swarm.parallel(sing,args_dict=sing_arg)



if __name__ == '__main__':
    URI1 = 'radio://0/30/2M/E7E7E7E70E'

    uris = {
        URI1,
    }

    cflib.crtp.init_drivers()
    with Swarm(uris, factory= CachedCfFactory(rw_cache='./cache')) as swarm:
        # swarm.parallel(sing)
        # swarm.parallel(sing)
        sing_swarm(uris,swarm,[100,100,100,100],500)
        # swarm.parallel(sing,args_dict={'freq':[400,329,392,523],'duration_ms':500})
        # swarm.parallel(wait_for_sing)


# C4    = 262
# DES4  = 277
# D4    = 294
# ES4   = 311
# E4    = 330
# F4    = 349
# GES4  = 370
# G4    = 392
# AS4   = 415
# A4    = 440
# B4    = 466
# H4    = 493
# C5    = 523
# DES5  = 554
# D5    = 587
# ES5   = 622
# E5    = 659
# F5    = 698
# GES5  = 740
# G5    = 783
# AS5   = 830
# A5    = 880
# B5    = 932
# H5    = 987
# C6    = 1046
# DES6  = 1108
# D6    = 1174
# ES6   = 1244
# E6    = 1318
# F6    = 1396
# GES6  = 1479
# G6    = 1567
# AS6   = 1661
# A6    = 1760
# B6    = 1864
# H6    = 1975
# C7    = 2093
# DES7  = 2217
# D7    = 2349
# ES7   = 2489
# E7    = 2637
# F7    = 2793
# GES7  = 2959
# G7    = 3135
# AS7   = 3322
# A7    = 3520
# H7    = 3729
# B7    = 3951