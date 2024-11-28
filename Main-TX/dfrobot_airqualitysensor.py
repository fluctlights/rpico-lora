from machine import Pin, I2C
import utime

class DFRobot_AirQualitySensor:
    # Select PM type
    PARTICLE_PM1_0_STANDARD   = 0x05
    PARTICLE_PM2_5_STANDARD   = 0x07
    PARTICLE_PM10_STANDARD    = 0x09
    PARTICLE_PM1_0_ATMOSPHERE = 0x0B
    PARTICLE_PM2_5_ATMOSPHERE = 0x0D
    PARTICLE_PM10_ATMOSPHERE  = 0x0F
    PARTICLENUM_0_3_UM_EVERY0_1L_AIR = 0X11
    PARTICLENUM_0_5_UM_EVERY0_1L_AIR = 0X13
    PARTICLENUM_1_0_UM_EVERY0_1L_AIR = 0X15
    PARTICLENUM_2_5_UM_EVERY0_1L_AIR = 0X17
    PARTICLENUM_5_0_UM_EVERY0_1L_AIR = 0X19
    PARTICLENUM_10_UM_EVERY0_1L_AIR  = 0X1B
    PARTICLENUM_GAIN_VERSION = 0x1D

    def test(self):
        print("Scanning I2C bus...")

        devices = self.i2c.scan()

        if devices:
            print("I2C devices found:")
            for device in devices:
                print(f" - Device found at address: {hex(device)}")
        else:
            print("No I2C devices found.")

    def __init__(self):
        self.i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)  # Pin 2 (SDA), Pin 3 (SCL), 100kHz
        self.__addr = 0x19

        self.test()

    def gain_particle_concentration_ugm3(self, PMtype):
        """Get PM concentration of a specified type."""
        buf = self.read_reg(PMtype, 2)
        print(buf)
        concentration = (buf[0] << 8) + buf[1]
        return concentration

    def gain_particlenum_every0_1l(self, PMtype):
        """Get the number of PM in 0.1L of air."""
        buf = self.read_reg(PMtype, 2)
        print(buf)
        particlenum = (buf[0] << 8) + buf[1]
        return particlenum

    def gain_version(self):
        """Get firmware version."""
        version = self.read_reg(self.PARTICLENUM_GAIN_VERSION, 1)
        return version[0] # type: ignore

    def set_lowpower(self):
        """Set the sensor to low-power mode."""
        self.write_reg(0x01, [0x01])

    def awake(self):
        """Wake up the sensor."""
        self.write_reg(0x01, [0x02])

    def write_reg(self, reg, data):
        """Write data to a register."""
        try:
            self.i2c.writeto_mem(self.__addr, reg, bytearray(data))
        except OSError as e:
            print("Write error:", e)

    def read_reg(self, reg, length):
        """Read data from a register."""
        try:
            return self.i2c.readfrom_mem(self.__addr, reg, length)
        except OSError as e:
            print("Read error:", e)
            return bytearray([0] * length)
