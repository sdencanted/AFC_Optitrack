from Filter import IIR2Filter
import math


class RealTimeProcessor(object):
    def __init__(self, order, cutoff, ftype, design, rs, sample_rate):
        self.flag = 0
        self.sample_time = 1 / sample_rate
        self.sample_rate = sample_rate

        # raw data from motion capture
        self.px1 = 0
        self.py1 = 0
        self.pz1 = 0
        self.quat_x = 0
        self.quat_y = 0
        self.quat_w = 0
        self.quat_z = 0

        # filtered data with IIR2Filter
        self.px_filtered = 0
        self.py_filtered = 0
        self.pz_filtered = 0
        self.quat_x_filtered = 0
        self.quat_y_filtered = 0
        self.quat_z_filtered = 0
        self.quat_w_filtered = 0

        # component of rotation matrix
        self.R11 = 0
        self.R12 = 0
        self.R13 = 0
        self.R21 = 0
        self.R22 = 0
        self.R23 = 0
        self.R31 = 0
        self.R32 = 0
        self.R33 = 0


        # filter setup
        self.FilterX = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterY = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterZ = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterQX = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterQY = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterQZ = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterQW = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)


    def data_unpack_vrpn(self, position,quaternion):

        self.px1 = position[0]   # position px
        self.py1 = position[1]   # position py
        self.pz1 = position[2]   # position pz
        self.quat_x = float(quaternion[0])
        self.quat_y =float( quaternion[1])
        self.quat_z = float(quaternion[2])
        self.quat_w = float(quaternion[3])

        self.raw_data = [self.px1, self.py1, self.pz1, self.quat_x, self.quat_y, self.quat_z, self.quat_w]
    def get_data_filtered(self):

        self.px_filtered = self.FilterX.filter(self.px1)
        self.py_filtered = self.FilterY.filter(self.py1)
        self.pz_filtered = self.FilterZ.filter(self.pz1)

        self.quat_x_filtered = self.FilterQX.filter(self.quat_x)
        self.quat_y_filtered = self.FilterQY.filter(self.quat_y)
        self.quat_z_filtered = self.FilterQZ.filter(self.quat_z)
        self.quat_w_filtered = self.FilterQW.filter(self.quat_w)
        filtered_data = [self.px_filtered, self.py_filtered, self.pz_filtered, self.quat_x_filtered, self.quat_y_filtered,
                       self.quat_z_filtered, self.quat_w_filtered]

        return filtered_data

    def get_rotm(self):

        xx1 = self.quat_x * self.quat_x
        yy1 = self.quat_y * self.quat_y
        zz1 = self.quat_z * self.quat_z
        xy1 = self.quat_x * self.quat_y
        xz1 = self.quat_x * self.quat_z
        yz1 = self.quat_y * self.quat_z
        wx1 = self.quat_w * self.quat_x
        wy1 = self.quat_w * self.quat_y
        wz1 = self.quat_w * self.quat_z

        self.R11 = 1 - 2 * (yy1 + zz1)
        self.R12 = 2 * (xy1 - wz1)
        self.R13 = 2 * (xz1 + wy1)
        self.R21 = 2 * (xy1 + wz1)
        self.R22 = 1 - 2 * (xx1 + zz1)
        self.R23 = 2 * (yz1 - wx1)
        self.R31 = 2 * (xz1 - wy1)
        self.R32 = 2 * (yz1 + wx1)
        self.R33 = 1 - 2 * (xx1 + yy1)

        rotm = [self.R11, self.R12, self.R13, self.R21, self.R22, self.R23, self.R31, self.R32, self.R33]
        return rotm


    def get_heading_x(self):
        heading = math.atan2(self.R21, self.R11)
        return heading

    def get_state(self):
        
        yaw = self.get_heading_x()
        # position feedback
        state= [self.px1, self.py1, self.pz1, self.quat_x, self.quat_y, self.quat_z, self.quat_w, yaw]
        return state







