#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "DFRobot_AirQualitySensor.hpp"

// I2C defines
#define I2C_PORT i2c0
#define I2C_SDA 0 //GPIO PINS, NOT THE PIN NUMBERS!
#define I2C_SCL 1
#define I2C_ADDR 0x19 //ADDR

void init_i2c()
{
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void look_for_addr()
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
int read_register(uint8_t reg, uint8_t *data, size_t len) 
{
    int result = i2c_write_blocking(I2C_PORT, I2C_ADDR, &reg, 1, true);
    if (result != 1) {
        return -1;  // Failed to write register address
    }

    // Read the requested data
    result = i2c_read_blocking(I2C_PORT, I2C_ADDR, data, len, false);
    if (result < 0) {
        return -2;  // Failed to read data
    }
    return result;  // Success, return number of bytes read
}

// Write data from a specific register
int write_register(uint8_t reg, const uint8_t *data, size_t len) {
    uint8_t buffer[len + 1];
    buffer[0] = reg;             // The first byte is the register address
    for (size_t i = 0; i < len; i++) {
        buffer[i + 1] = data[i]; // The rest are the data bytes
    }

    // Write the buffer to the device
    int result = i2c_write_blocking(I2C_PORT, I2C_ADDR, buffer, len + 1, false);
    if (result < 0) {
        DBG("Failed to write to register 0x%02X\n", reg);
        return -1; // Return error
    }
    return result; // Return the number of bytes written
}

int gain_version()
{
    uint8_t sensor_version = 0;
    int result = read_register(PARTICLENUM_GAIN_VERSION, &sensor_version, 1); // DRROBOT FW version, register addr
    
    if (result > 0) {
        return sensor_version;
    } else {
        printf("Failed to read from device. Error: %d\n", result);
        return -1;
    }
    
}

int main()
{
    stdio_init_all();

    init_i2c();
    look_for_addr();

    while (1) {
        int version = gain_version();
        printf("Version is: %d\n", version);
        sleep_ms(1000);  // Wait 1 second between readings
    }









}
