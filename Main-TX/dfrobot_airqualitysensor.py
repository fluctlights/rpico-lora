from machine import Pin, I2C
import utime

class DFRobot_AirQualitySensor:
    # Select PM type
    PARTICLE_PM1_0_STANDARD   = 0X05
    PARTICLE_PM2_5_STANDARD   = 0X07
    PARTICLE_PM10_STANDARD    = 0X09
    PARTICLE_PM1_0_ATMOSPHERE = 0X0B
    PARTICLE_PM2_5_ATMOSPHERE = 0X0D
    PARTICLE_PM10_ATMOSPHERE  = 0X0F
    PARTICLENUM_0_3_UM_EVERY0_1L_AIR = 0X11
    PARTICLENUM_0_5_UM_EVERY0_1L_AIR = 0X13
    PARTICLENUM_1_0_UM_EVERY0_1L_AIR = 0X15
    PARTICLENUM_2_5_UM_EVERY0_1L_AIR = 0X17
    PARTICLENUM_5_0_UM_EVERY0_1L_AIR = 0X19
    PARTICLENUM_10_UM_EVERY0_1L_AIR  = 0X1B
    PARTICLENUM_GAIN_VERSION = 0X1D

    def test(self):
        utime.sleep(10)
        print("Scanning I2C bus...")

        devices = self.i2c.scan()

        if devices:
            print("I2C devices found:")
            for device in devices:
                print(f" - Device found at address: {hex(device)}")
        else:
            print("No I2C devices found.")

    def __init__(self):
        self.i2c = I2C(1, scl=Pin(3), sda=Pin(2), freq=100000)  # Pin 2 (SDA), Pin 3 (SCL), 100kHz
        self.addr = 0x19

        self.test()

    def gain_particle_concentration_ugm3(self, PMtype):
        """Get PM concentration of a specified type."""
        buf = self.read_reg(PMtype, 2)
        concentration = (buf[0] << 8) + buf[1]
        return concentration

    def gain_particlenum_every0_1l(self, PMtype):
        """Get the number of PM in 0.1L of air."""
        buf = self.read_reg(PMtype, 2)
        particlenum = (buf[0] << 8) + buf[1]
        return particlenum

    def gain_version(self):
        """Get firmware version."""
        version = self.read_reg(self.PARTICLENUM_GAIN_VERSION, 1)
        return version[0]

    def set_lowpower(self):
        """Set the sensor to low-power mode."""
        self.write_reg(0x01, [0x01])

    def awake(self):
        """Wake up the sensor."""
        self.write_reg(0x01, [0x02])

    def write_reg(self, reg, data):
        """
        Write data to a register.
        :param reg: The register address to write to.
        :param data: A list or bytearray of data to write.
        """
        try:
            # Concatenate the register and data into a single bytearray
            self.i2c.writeto(self.addr, bytearray([reg] + data))
        except OSError as e:
            print("Write error:", e)

    def read_reg(self, reg, length):
        """
        Read data from a register.
        :param reg: The register address to read from.
        :param length: The number of bytes to read.
        :return: A bytearray of read data.
        """
        try:
            # Write the register address first
            self.i2c.writeto(self.addr, bytearray([reg]))
            # Then read the specified number of bytes
            return self.i2c.readfrom(self.addr, length)
        except OSError as e:
            print("Read error:", e)
            return bytearray([0] * length)

airquality_sensor = DFRobot_AirQualitySensor()
utime.sleep_ms(3000)
airquality_sensor.awake()
utime.sleep_ms(3000)
version = airquality_sensor.gain_version()
print("Firmware version is: " + str(version))
utime.sleep_ms(3000)
num_particles_bigger_than_2_5_um_per_0_1_l = airquality_sensor.gain_particlenum_every0_1l(airquality_sensor.PARTICLENUM_2_5_UM_EVERY0_1L_AIR)
utime.sleep_ms(3000)
concentration_pm2_5_in_ug_m3 = airquality_sensor.gain_particle_concentration_ugm3(airquality_sensor.PARTICLE_PM2_5_STANDARD)
