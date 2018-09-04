# mpu9250.py MicroPython driver for the InvenSense MPU9250 inertial measurement unit
# Authors Peter Hinch, Sebastian Plamauer
# V0.5 17th June 2015

'''
mpu9250 is a micropython module for the InvenSense MPU9250 sensor.
It measures acceleration, turn rate and the magnetic field in three axis.

The MIT License (MIT)
Copyright (c) 2014 Sebastian Plamauer, oeplse@gmail.com, Peter Hinch
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
'''

from .imu import InvenSenseMPU, bytes_toint, MPUException
from .vector3d import Vector3d
import pigpio

class MPU9250(InvenSenseMPU):
    '''
    MPU9250 constructor arguments
    1. side_str 'X' or 'Y' depending on the Pyboard I2C interface being used
    2. optional device_addr 0, 1 depending on the voltage applied to pin AD0 (Drotek default is 1)
       if None driver will scan for a device (if one device only is on bus)
    3, 4. transposition, scaling optional 3-tuples allowing for outputs to be based on vehicle
          coordinates rather than those of the sensor itself. See readme.
    '''

    _mpu_addr = (104, 105)  # addresses of MPU9250 determined by voltage on pin AD0
    _mag_addr = 12          # Magnetometer address
    _chip_id = 113

    def __init__(self, side_str, device_addr=None, transposition=(0, 1, 2), scaling=(1, 1, 1)):

        super().__init__(side_str, device_addr, transposition, scaling)
        self.m_mpu_i2c = self.pi.i2c_open(1, 12)

        self._mag = Vector3d(transposition, scaling, self._mag_callback)
        self.accel_filter_range = 0             # fast filtered response
        self.gyro_filter_range = 0
        self._mag_stale_count = 0               # MPU9250 count of consecutive reads where old data was returned
        self.mag_correction = self._magsetup()  # 16 bit, 100Hz update.Return correction factors.
        self._mag_callback()  # Seems neccessary to kick the mag off else 1st reading is zero (?)


    @property
    def sensors(self):
        '''
        returns sensor objects accel, gyro and mag
        '''
        return self._accel, self._gyro, self._mag

    # get temperature
    @property
    def temperature(self):
        '''
        Returns the temperature in degree C.
        '''
        try:
            self.buf2=self._read(0x41, 2)
        except OSError:
            raise MPUException(self._I2Cerror)
        return bytes_toint(self.buf2[0], self.buf2[1])/333.87 + 21  # I think

    # Low pass filters
    @property
    def gyro_filter_range(self):
        '''
        Returns the gyro and temperature sensor low pass filter cutoff frequency
        Pass:               0   1   2   3   4   5   6   7
        Cutoff (Hz):        250 184 92  41  20  10  5   3600
        Sample rate (KHz):  8   1   1   1   1   1   1   8
        '''
        try:
            self.buf1 = self._read(0x1A, 1)
            res = self.buf1[0] & 7
        except OSError:
            raise MPUException(self._I2Cerror)
        return res

    @gyro_filter_range.setter
    def gyro_filter_range(self, filt):
        '''
        Sets the gyro and temperature sensor low pass filter cutoff frequency
        Pass:               0   1   2   3   4   5   6   7
        Cutoff (Hz):        250 184 92  41  20  10  5   3600
        Sample rate (KHz):  8   1   1   1   1   1   1   8
        '''
        if filt in range(8):
            try:
                self._write(0x1A,filt)
            except OSError:
                raise MPUException(self._I2Cerror)
        else:
            raise ValueError('Filter coefficient must be between 0 and 7')

    @property
    def accel_filter_range(self):
        '''
        Returns the accel low pass filter cutoff frequency
        Pass:               0   1   2   3   4   5   6   7 BEWARE 7 doesn't work on device I tried.
        Cutoff (Hz):        460 184 92  41  20  10  5   460
        Sample rate (KHz):  1   1   1   1   1   1   1   1
        '''
        try:
            self.buf1 = self._read(0x1D, 1)
            res = self.buf1[0] & 7
        except OSError:
            raise MPUException(self._I2Cerror)
        return res

    @accel_filter_range.setter
    def accel_filter_range(self, filt):
        '''
        Sets the accel low pass filter cutoff frequency
        Pass:               0   1   2   3   4   5   6   7 BEWARE 7 doesn't work on device I tried.
        Cutoff (Hz):        460 184 92  41  20  10  5   460
        Sample rate (KHz):  1   1   1   1   1   1   1   1
        '''
        if filt in range(8):
            try:
                self._write(0x1D,filt)
            except OSError:
                raise MPUException(self._I2Cerror)
        else:
            raise ValueError('Filter coefficient must be between 0 and 7')

    def _magsetup(self):                        # mode 2 100Hz continuous reads, 16 bit
        '''
        Magnetometer initialisation: use 16 bit continuous mode.
        Mode 1 is 8Hz mode 2 is 100Hz repetition
        returns correction values
        '''
        try:
            self.m_write(0x0A,0x0F)      # fuse ROM access mode
            self.buf3 = self.m_read( 0x10, 3)  # Correction values
            self.m_write(0x0A,0)         # Power down mode (AK8963 manual 6.4.6)
            self.m_write(0x0A, 0x16)      # 16 bit (0.15uT/LSB not 0.015), mode 2
        except OSError:
            raise MPUException(self._I2Cerror)
        mag_x = (0.5*(self.buf3[0] - 128))/128 + 1
        mag_y = (0.5*(self.buf3[1] - 128))/128 + 1
        mag_z = (0.5*(self.buf3[2] - 128))/128 + 1
        return (mag_x, mag_y, mag_z)

    @property
    def mag(self):
        '''
        Magnetomerte object
        '''
        return self._mag

    def _mag_callback(self):
        '''
        Update magnetometer Vector3d object (if data available)
        '''
        try:                                    # If read fails, returns last valid data and
            buf1 = self.m_read(0x02, 1)  # increments mag_stale_count
            if buf1[0] & 1 == 0:
                return self._mag                # Data not ready: return last value
            buf6 = self.m_read(0x03, 6)
            buf1 = self.m_read(0x09, 1)
        except OSError:
            raise MPUException(self._I2Cerror)
       # if buf1[0] & 0x08 > 0:             # An overflow has occurred
        #    self._mag_stale_count += 1          # Error conditions retain last good value
        #    return                              # user should check for ever increasing stale_counts
        self._mag._ivector[1] = bytes_toint(buf6[1], buf6[0])  # Note axis twiddling and little endian
        self._mag._ivector[0] = bytes_toint(buf6[3], buf6[2])
        self._mag._ivector[2] = -bytes_toint(buf6[5], buf6[4])
        scale = 0.15                            # scale is 0.15uT/LSB
        self._mag._vector[0] = self._mag._ivector[0]*self.mag_correction[0]*scale
        self._mag._vector[1] = self._mag._ivector[1]*self.mag_correction[1]*scale
        self._mag._vector[2] = self._mag._ivector[2]*self.mag_correction[2]*scale
        self._mag_stale_count = 0

    @property
    def mag_stale_count(self):
        '''
        Number of consecutive times where old data was returned
        '''
        return self._mag_stale_count

    def get_mag_irq(self):
        '''
        Uncorrected values because floating point uses heap
        '''
        self.buf1 = self.m_read(0x02, 1)
        if self.buf1[0] == 1:                   # Data is ready
            self.buf6 = self.m_read(0x03, 6)
            self.buf1 = self.m_read(0x09, 1)    # Mandatory status2 read
            self._mag._ivector[1] = 0
            if self.buf1[0] & 0x08 == 0:        # No overflow has occurred
                self._mag._ivector[1] = bytes_toint(self.buf6[1], self.buf6[0])
                self._mag._ivector[0] = bytes_toint(self.buf6[3], self.buf6[2])
                self._mag._ivector[2] = -bytes_toint(self.buf6[5], self.buf6[4])

    def m_read(self, memaddr,count):        # addr = I2C device address, memaddr = memory location within the I2C device
        '''
        Read bytes to pre-allocated buffer Caller traps OSError.
        '''
        count1, bufy = self.pi.i2c_read_i2c_block_data(self.m_mpu_i2c, memaddr, count)

        if count == count1:
            return bufy
        else:
            raise MPUException(self._I2Cerror)

    # write to device
    def m_write(self, memaddr, data):
        '''
        Perform a memory write. Caller should trap OSError.
        '''
        if isinstance(data, int):
            self.pi.i2c_write_byte_data(self.m_mpu_i2c, memaddr, data)
        else:
            self.pi.i2c_write_i2c_block_data(self.m_mpu_i2c, memaddr, data)
