#include "DFRobot_AirQualitySensor.hpp"

DFRobot_AirQualitySensor::DFRobot_AirQualitySensor(uint8_t i2c_addr, uint8_t i2c_sda, uint8_t i2c_scl, i2c_inst_t* i2c_port)
{
  this->_i2c_addr = i2c_addr;
  this->_i2c_sda = i2c_sda;
  this->_i2c_scl = i2c_scl;
  this->_i2c_port = *i2c_port;
  
  init_i2c();
  look_for_addr();
}

void DFRobot_AirQualitySensor::init_i2c()
{
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(&_i2c_port, 400*1000);
    gpio_set_function(this->_i2c_sda, GPIO_FUNC_I2C);
    gpio_set_function(this->_i2c_scl, GPIO_FUNC_I2C);
    gpio_pull_up(this->_i2c_sda);
    gpio_pull_up(this->_i2c_scl);
}

bool DFRobot_AirQualitySensor::reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void DFRobot_AirQualitySensor::look_for_addr()
{
    // First, getting if there is any I2C device connected
    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);

        //printf(ret < 0 ? "." : "@"); //another option
        printf(ret < 0 ? "." : "%d", addr);
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
}

// Read data from a specific register
int16_t DFRobot_AirQualitySensor::read_register(uint8_t reg, uint8_t *data, uint8_t len) 
{
    int16_t result = i2c_write_blocking(&_i2c_port, this->_i2c_addr, &reg, 1, true);
    if (result != 1) {
        return -1;  // Failed to write register address
    }

    // Read the requested data
    result = i2c_read_blocking(&_i2c_port, this->_i2c_addr, data, len, false);
    if (result < 0) {
        return -2;  // Failed to read data
    }
    return result;  // Success, return number of bytes read
}

// Write data from a specific register
void DFRobot_AirQualitySensor::write_register(uint8_t reg, const uint8_t *data, uint8_t len) {
    uint8_t buffer[len + 1];
    buffer[0] = reg;             // The first byte is the register address
    for (size_t i = 0; i < len; i++) {
        buffer[i + 1] = data[i]; // The rest are the data bytes
    }

    // Write the buffer to the device
    int result = i2c_write_blocking(&_i2c_port, this->_i2c_addr, buffer, len + 1, false);
    if (result < 0) {
        printf("Failed to write to register 0x%02X\n", reg);
    }
}

uint8_t DFRobot_AirQualitySensor::get_version()
{
    uint8_t sensor_version = 0;
    int result = read_register(SENSOR_VERSION, &sensor_version, 1); // DRROBOT FW version, register addr
    
    if (result < 0) {
        printf("Failed to read from device. Error: %d\n", result);
        return -1;
    }
    return sensor_version;
}

uint16_t DFRobot_AirQualitySensor::gainParticleConcentration_mgm3(uint8_t type)
{
    uint8_t buf[2] = {0};
    uint16_t concentration;
    int result = read_register(type, buf, 2); 
    if (result < 2) {
        printf("I2C Read failed or incomplete: result=%d\n", result);
        return -1; 
    }
    concentration = ((uint16_t) buf[0] << 8) + (uint16_t) buf[1];
    return concentration;
}

uint16_t DFRobot_AirQualitySensor::gainParticleNum_Every0_1L(uint8_t type)
{
    uint8_t buf[2] = {0};
    uint16_t particlenum;
    int result = read_register(type, buf, 2);
    if (result < 2) {
        printf("I2C Read failed or incomplete: result=%d\n", result);
        return -1;
    }
    particlenum = ((uint16_t) buf[0] << 8) + (uint16_t) buf[1];
    return particlenum;
}

void DFRobot_AirQualitySensor::lowpower()
{
    uint8_t mode = 1;
    write_register(SENSOR_STATE, &mode, 1);
}

void DFRobot_AirQualitySensor::awake()
{
    uint8_t mode = 2;
    write_register(SENSOR_STATE, &mode, 1);
}