import math
import numpy as np
import numpy.linalg as la


class trajectory_generator(object):
    def __init__(self):
        pass

    def hover_test(self, y_offset):
        ref_x = 0
        ref_y = 0.6
        ref_z = 0.2
        ref_pos = np.array([ref_x, ref_y+y_offset, ref_z])
        msg = "hovering test..."
        return (ref_pos,msg)
    
    def simple_rectangle(self, y_offset, abs_time):
        if abs_time < 3:
            ref_x = 0
            ref_y = 0.6
            ref_z = 1.0
            ref_pos = np.array([ref_x, ref_y+y_offset, ref_z])
            msg = "still flying..."
        elif 3 <= abs_time < 7:
            ref_x = 0
            ref_y = 0.9
            ref_z = 1.0
            ref_pos = np.array([ref_x, ref_y+y_offset, ref_z])
            msg = "still flying..."
        elif 7 <= abs_time < 11:
            ref_x = 0
            ref_y = 1.2
            ref_z = 1.0
            ref_pos = np.array([ref_x, ref_y+y_offset, ref_z]) 
            msg = "still flying..." 
        elif 11 <= abs_time < 15:
            ref_x = 0.5
            ref_y = 1.2
            ref_z = 1.0
            ref_pos = np.array([ref_x, ref_y+y_offset, ref_z])
            msg = "still flying..."
        elif 15 <= abs_time < 19:
            ref_x = 0.5
            ref_y = 0.9
            ref_z = 1.0
            ref_pos = np.array([ref_x, ref_y+y_offset, ref_z])
            msg = "still flying..."
        elif 19 <= abs_time < 23:
            ref_x = 0.5
            ref_y = 0.6
            ref_z = 1.0
            ref_pos = np.array([ref_x, ref_y+y_offset, ref_z])
            msg = "still flying..."
        elif 23 <= abs_time < 27:
            ref_x = 0.5
            ref_y = 0.6
            ref_z = 0.2
            ref_pos = np.array([ref_x, ref_y+y_offset, ref_z])
            msg = "still flying..."
        elif 27 <= abs_time:
            ref_x = 0.5
            ref_y = 0.6
            ref_z = 0.02
            ref_pos = np.array([ref_x, ref_y+y_offset, ref_z])
            msg = "traj ended..."

        return (ref_pos,msg)


    