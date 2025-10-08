#load a midi file and convert to freq and duration
#import a midi library
import time
from mido import MidiFile
import test_sing

import cflib.crtp
import numpy as np
import time
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
def midi_to_freq_duration(uris,swarm,midi_file,initial_octave_offset=0):
    mid = MidiFile(midi_file)
    notes = []
    duration_ms = 0

    # keep track of notes that are on in `notes`.
    # when any not is turned on or off, we will calculate the time since 
    # the last change and
    # print the note and duration

    # shift all notes by freq_offset if frequency falls below 262 or above 3951
    freq_offset=initial_octave_offset*12
    accumulated_time=0
    for msg in mid:
        #check if msg has time attribute
        if not hasattr(msg, 'time'):
            continue
        elif msg.is_meta or (msg.type not in ('note_on', 'note_off')) or msg.note+freq_offset<48:
            if msg.type  in ('note_on', 'note_off') and msg.note+freq_offset<48:
                print("note too low, skipping",msg.note+freq_offset)
            else:
                print("meta or non note event, skipping")
            accumulated_time+=msg.time
            continue
        duration_ms = round((msg.time+accumulated_time) * 1000)  # convert to milliseconds
        time_start=time.time()
        if duration_ms > 0:
            print([note+12 for note in notes],duration_ms)
            test_sing.sing_swarm(uris,swarm,[round(440 * (2 ** ((note - 69+freq_offset) / 12))) for note in notes],duration_ms)
        
        note_adjusted=msg.note+freq_offset
        if note_adjusted<60 or note_adjusted>107:
            while note_adjusted<60:
                freq_offset+=12
                note_adjusted=msg.note+freq_offset
            while note_adjusted>107:
                freq_offset-=12
                note_adjusted=msg.note+freq_offset
    
        if msg.type == 'note_on' and msg.velocity > 0:
            # print("adding",msg.note)
            if msg.note not in notes:
                
                # assert msg.note not in notes, f"Note {msg.note} already in notes list {notes}"
                notes.append(msg.note)
        elif msg.type in ('note_off', 'note_on') and msg.velocity == 0:
            # note off event
            # remove the note from notes
            # print("removing",msg.note)
            if msg.note in notes:
                notes.remove(msg.note)
        time_sleep=msg.time+accumulated_time-(time.time()-time_start)
        accumulated_time=0
        if time_sleep>0:
            time.sleep(time_sleep)
    test_sing.sing_swarm(uris,swarm,[],100)
    return 
#example usage
midi_file = '/home/painis/Downloads/umapyoi densetsu.mid'
# midi_file = 'furelise.mid'

URI1 = 'radio://0/30/2M/E7E7E7E70E'
URI2 = 'radio://0/30/2M/E7E7E7E707'
URI3 = 'radio://0/30/2M/E7E7E7E708'
URI4 = 'radio://1/30/2M/E7E7E7E709'
URI5 = 'radio://1/30/2M/E7E7E7E710'

uris = {
    URI1,
    URI2,
    URI3,
    URI4,
    URI5
}

cflib.crtp.init_drivers()
with Swarm(uris, factory= CachedCfFactory(rw_cache='./cache')) as swarm:
    # print(Swarm)
    # test_sing.sing_swarm(uris,swarm,[480,480+80,480-80,480],4000)
    # time.sleep(4)
    # exit()
    # for i in range(254,500,10):
    #     print(i)
    #     for _ in range(10):
    #         timenow=time.time()
    #         test_sing.sing_swarm(uris,swarm,[i,i+80,i-80,i],40)
    #         time.sleep(max(0,0.04-(time.time()-timenow)))
    #     # time.sleep(1)

    # test_sing.sing_swarm(uris,swarm,[round(440 * (2 ** ((84 - 69) / 12)))],1250)
    # time.sleep(1)
    midi_to_freq_duration(uris,swarm,midi_file,0)