# imu.py MicroPython driver for the InvenSense inertial measurement units
# This is the base class
# Adapted from Sebastian Plamauer's MPU9150 driver:
# https://github.com/micropython-IMU/micropython-mpu9150.git
# Authors Peter Hinch, Sebastian Plamauer
# V0.1 13th June 2015 Experimental: this code is not yet fully tested

'''
mpu9250 is a micropython module for the InvenSense MPU9250 sensor.
It measures acceleration, turn rate and the magnetic field in three axis.
mpu9150 driver modified for the MPU9250 by Peter Hinch

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

# User access is now by properties e.g.
# myimu = MPU9250('X')
# magx = myimu.mag.x
# accelxyz = myimu.accel.xyz
# Error handling: on code used for initialisation, abort with message
# At runtime try to continue returning last good data value. We don't want aircraft
# crashing. However if the I2C has crashed we're probably stuffed.

#import pyb
import pigpio
from .vector3d import Vector3d
from time import sleep

class MPUException(OSError):
    '''
    Exception for MPU devices
    '''

    pass


def bytes_toint(msb, lsb):
    '''
    Convert two bytes to signed integer (big endian)
    for little endian reverse msb, lsb arguments
    Can be used in an interrupt handler
    '''
    if not msb & 0x80:
        return msb << 8 | lsb  # +ve
    return - (((msb ^ 255) << 8) | (lsb ^ 255) + 1)


class InvenSenseMPU(object):
    '''
    Module for InvenSense 9DOF IMUs. Base class implements features common to MPU9150 and MPU9250.
    '''

    _I2Cerror = "I2C failure when communicating with IMU"

    def __init__(self, side_str, device_addr, transposition, scaling):

        self._accel = Vector3d(transposition, scaling, self._accel_callback)
        self._gyro = Vector3d(transposition, scaling, self._gyro_callback)
        self.buf1 = bytearray([0]*1)            # Pre-allocated buffers for reads: allows reads to
        self.buf2 = bytearray([0]*2)            # be done in interrupt handlers
        self.buf3 = bytearray([0]*3)
        self.buf6 = bytearray([0]*6)
        self.timeout = 10                       # I2C tieout mS

        sleep(0.2)
        pi=pigpio.pi()
        self._mpu_i2c = pi.i2c_open(1, 104)
        self.pi=pi

        self.chip_id                     # Test communication by reading chip_id: throws exception on error
        # Can communicate with chip. Set it up.
        self.wake()                             # wake it up
        self.passthrough = True                 # Enable mag access from main I2C bus
        self.accel_range = 0                    # default to highest sensitivity
        self.gyro_range = 0                     # Likewise for gyro

    # read from device
    def _read(self, memaddr,count):        # addr = I2C device address, memaddr = memory location within the I2C device
        '''
        Read bytes to pre-allocated buffer Caller traps OSError.
        '''
        count1, bufy = self.pi.i2c_read_i2c_block_data(self._mpu_i2c, memaddr, count)

        if count == count1:
            return bufy
        else:
            raise MPUException(self._I2Cerror)

    # write to device
    def _write(self, memaddr, data):
        '''
        Perform a memory write. Caller should trap OSError.
        '''
        if isinstance(data, int):
            self.pi.i2c_write_byte_data(self._mpu_i2c, memaddr, data)
        else:
            self.pi.i2c_write_i2c_block_data(self._mpu_i2c, memaddr, data)

    # wake
    def wake(self):
        '''
        Wakes the device.
        '''
        try:
            self._write( 0x6B, 0x01)  # Use best clock source
        except OSError:
            raise MPUException(self._I2Cerror)
        return 'awake'

    # mode
    def sleep(self):
        '''
        Sets the device to sleep mode.
        '''
        try:
            self._write( 0x6B, 0x40)
        except OSError:
            raise MPUException(self._I2Cerror)
        return 'asleep'

    # chip_id
    @property
    def chip_id(self):
        '''
        Returns Chip ID
        '''
        try:
            self.buf1=self._read(0x75, 1)
        except OSError:
            raise MPUException(self._I2Cerror)
        chip_id = int(self.buf1[0])
        if chip_id != self._chip_id:
            raise ValueError('Bad chip ID retrieved: MPU communication failure')
        return chip_id

    # passthrough
    @property
    def passthrough(self):
        '''
        Returns passthrough mode True or False
        '''
        try:
            self.buf1 = self._read(0x37, 1)
            return self.buf1[0] & 0x02 > 0
        except OSError:
            raise MPUException(self._I2Cerror)

    @passthrough.setter
    def passthrough(self, mode):
        '''
        Sets passthrough mode True or False
        '''
        if type(mode) is bool:
            val = 2 if mode else 0
            try:
                self._write(0x37,val)  # I think this is right.
                self._write(0x6A,0x00)
            except OSError:
                raise MPUException(self._I2Cerror)
        else:
            raise ValueError('pass either True or False')

    # sample rate. Not sure why you'd ever want to reduce this from the default.
    @property
    def sample_rate(self):
        '''
        Get sample rate as per Register Map document section 4.4
        SAMPLE_RATE= Internal_Sample_Rate / (1 + rate)
        default rate is zero i.e. sample at internal rate.
        '''
        try:
            self.buf1 = self._read(0x19, 1)
            return self.buf1[0]
        except OSError:
            raise MPUException(self._I2Cerror)

    @sample_rate.setter
    def sample_rate(self, rate):
        '''
        Set sample rate as per Register Map document section 4.4
        '''
        if rate < 0 or rate > 255:
            raise ValueError("Rate must be in range 0-255")
        try:
            self._write(0x19,rate)
        except OSError:
            raise MPUException(self._I2Cerror)

    # accelerometer range
    @property
    def accel_range(self):
        '''
        Accelerometer range
        Value:              0   1   2   3
        for range +/-:      2   4   8   16  g
        '''
        try:
            self.buf1= self._read(0x1C, 1)
            ari = self.buf1[0]//8
        except OSError:
            raise MPUException(self._I2Cerror)
        return ari

    @accel_range.setter
    def accel_range(self, accel_range):
        '''
        Set accelerometer range
        Pass:               0   1   2   3
        for range +/-:      2   4   8   16  g
        '''
        ar_bytes = (0x00, 0x08, 0x10, 0x18)
        if accel_range in range(len(ar_bytes)):
            try:
                self._write(0x1C,ar_bytes[accel_range] )
            except OSError:
                raise MPUException(self._I2Cerror)
        else:
            raise ValueError('accel_range can only be 0, 1, 2 or 3')

    # gyroscope range
    @property
    def gyro_range(self):
        '''
        Gyroscope range
        Value:              0   1   2    3
        for range +/-:      250 500 1000 2000  degrees/second
        '''
        # set range
        try:
            self.buf1= self._read(0x1B, 1)
            gri = self.buf1[0]//8
        except OSError:
            raise MPUException(self._I2Cerror)
        return gri

    @gyro_range.setter
    def gyro_range(self, gyro_range):
        '''
        Set gyroscope range
        Pass:               0   1   2    3
        for range +/-:      250 500 1000 2000  degrees/second
        '''
        gr_bytes = (0x00, 0x08, 0x10, 0x18)
        if gyro_range in range(len(gr_bytes)):
            try:
                self._write(0x1B,gr_bytes[gyro_range])  # Sets fchoice = b11 which enables filter
            except OSError:
                raise MPUException(self._I2Cerror)
        else:
            raise ValueError('gyro_range can only be 0, 1, 2 or 3')

    # Accelerometer
    @property
    def accel(self):
        '''
        Acceleremoter object
        '''
        return self._accel

    def _accel_callback(self):
        '''
        Update accelerometer Vector3d object
        '''
        try:
            self.buf6= self._read(0x3B, 6)
        except OSError:
            raise MPUException(self._I2Cerror)
        self._accel._ivector[0] = bytes_toint(self.buf6[0], self.buf6[1])
        self._accel._ivector[1] = bytes_toint(self.buf6[2], self.buf6[3])
        self._accel._ivector[2] = bytes_toint(self.buf6[4], self.buf6[5])
        scale = (16384, 8192, 4096, 2048)
        self._accel._vector[0] = self._accel._ivector[0]/scale[self.accel_range]
        self._accel._vector[1] = self._accel._ivector[1]/scale[self.accel_range]
        self._accel._vector[2] = self._accel._ivector[2]/scale[self.accel_range]

    def get_accel_irq(self):
        '''
        For use in interrupt handlers. Sets self._accel._ivector[] to signed
        unscaled integer accelerometer values
        '''
        self.buf6=self._read(0x3B, 6)
        self._accel._ivector[0] = bytes_toint(self.buf6[0], self.buf6[1])
        self._accel._ivector[1] = bytes_toint(self.buf6[2], self.buf6[3])
        self._accel._ivector[2] = bytes_toint(self.buf6[4], self.buf6[5])

    # Gyro
    @property
    def gyro(self):
        '''
        Gyroscope object
        '''
        return self._gyro

    def _gyro_callback(self):
        '''
        Update gyroscope Vector3d object
        '''
        try:
            self.buf6=self._read(0x43, 6)
        except OSError:
            raise MPUException(self._I2Cerror)
        self._gyro._ivector[0] = bytes_toint(self.buf6[0], self.buf6[1])
        self._gyro._ivector[1] = bytes_toint(self.buf6[2], self.buf6[3])
        self._gyro._ivector[2] = bytes_toint(self.buf6[4], self.buf6[5])
        scale = (131, 65.5, 32.8, 16.4)
        self._gyro._vector[0] = self._gyro._ivector[0]/scale[self.gyro_range]
        self._gyro._vector[1] = self._gyro._ivector[1]/scale[self.gyro_range]
        self._gyro._vector[2] = self._gyro._ivector[2]/scale[self.gyro_range]

    def get_gyro_irq(self):
        '''
        For use in interrupt handlers. Sets self._gyro._ivector[] to signed
        unscaled integer gyro values. Error trapping disallowed.
        '''
        self.buf6= self._read(0x43, 6)
        self._gyro._ivector[0] = bytes_toint(self.buf6[0], self.buf6[1])
        self._gyro._ivector[1] = bytes_toint(self.buf6[2], self.buf6[3])
        self._gyro._ivector[2] = bytes_toint(self.buf6[4], self.buf6[5])
