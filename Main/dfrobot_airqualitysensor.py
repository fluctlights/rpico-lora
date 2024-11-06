# -*- coding: utf-8 -*
'''!
  @file DFRobot_AirQualitySensor.py
  @brief The sensor can obtain PM concentration in the air.
  @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author      PengKaixing(kaixing.peng@dfrobot.com)
  @version  V1.0.0
  @date  2021-11-23
  @url https://github.com/dfrobot/DFRobot_AirQualitySensor
'''
import time
from machine import Pin, I2C

class DFRobot_AirQualitySensor(object):
    # PM types
    PARTICLE_PM1_0_STANDARD = 0x05
    PARTICLE_PM2_5_STANDARD = 0x07
    PARTICLE_PM10_STANDARD = 0x09
    PARTICLE_PM1_0_ATMOSPHERE = 0x0B
    PARTICLE_PM2_5_ATMOSPHERE = 0x0D
    PARTICLE_PM10_ATMOSPHERE = 0x0F
    PARTICLENUM_0_3_UM_EVERY0_1L_AIR = 0x11
    PARTICLENUM_0_5_UM_EVERY0_1L_AIR = 0x13
    PARTICLENUM_1_0_UM_EVERY0_1L_AIR = 0x15
    PARTICLENUM_2_5_UM_EVERY0_1L_AIR = 0x17
    PARTICLENUM_5_0_UM_EVERY0_1L_AIR = 0x19
    PARTICLENUM_10_UM_EVERY0_1L_AIR = 0x1B
    PARTICLENUM_GAIN_VERSION = 0x1D

    def __init__(self, i2c, addr):
        self.__addr = addr
        self.i2c = i2c

    def gain_particle_concentration_ugm3(self, PMtype):
        '''Get PM concentration of a specified type'''
        buf = self.read_reg(PMtype, 2)
        concentration = (buf[0] << 8) + buf[1]
        return concentration

    def gain_particlenum_every0_1l(self, PMtype):
        '''Get the number of PM in 0.1L of air'''
        buf = self.read_reg(PMtype, 2)
        particlenum = (buf[0] << 8) + buf[1]
        return particlenum

    def gain_version(self):
        '''Get the firmware version of the sensor'''
        version = self.read_reg(self.PARTICLENUM_GAIN_VERSION, 1)
        return version[0]

    def set_lowpower(self):
        '''Set the sensor to low-power mode'''
        mode = [0x01]
        self.write_reg(0x01, mode)

    def awake(self):
        '''Wake up the sensor'''
        mode = [0x02]
        self.write_reg(0x01, mode)

    def write_reg(self, reg, data):
        '''Write data to a register'''
        try:
            self.i2c.writeto_mem(self.__addr, reg, bytes(data))
        except:
            print("Error writing to register. Please check connections!")
            time.sleep(1)

    def read_reg(self, reg, length):
        '''Read data from a register'''
        try:
            return self.i2c.readfrom_mem(self.__addr, reg, length)
        except:
            print("Error reading from register. Please check connections!")
            return [-1] * length


# Initialize I2C for Raspberry Pi Pico
i2c = I2C(1, scl=Pin(27), sda=Pin(26), freq=100000)  # Pin 26 (SDA), Pin 27 (SCL), 100kHz

# Initialize sensor
I2C_ADDRESS = 0x19  # Default I2C address for sensor
sensor = DFRobot_AirQualitySensor(i2c, I2C_ADDRESS)

# Setup
def setup():
    time.sleep(1)
    # Get firmware version
    version = sensor.gain_version()
    print("Firmware version is: " + str(version))
    time.sleep(1)

# Main loop
def loop():
    # Get PM2.5 concentration
    concentration = sensor.gain_particle_concentration_ugm3(sensor.PARTICLE_PM2_5_STANDARD)
    print("PM2.5 concentration (µg/m³): " + str(concentration))

    # Get particle count for 0.3µm particles per 0.1L of air
    num_particles = sensor.gain_particlenum_every0_1l(sensor.PARTICLENUM_0_3_UM_EVERY0_1L_AIR)
    print("Particles with a diameter of 0.3µm per 0.1L: " + str(num_particles))

    time.sleep(1)

# Run setup and loop
setup()
while True:
    loop()


    
