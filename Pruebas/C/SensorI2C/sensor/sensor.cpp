#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "DFRobot_AirQualitySensor.hpp"

// I2C defines
#define I2C_PORT i2c0
#define I2C_SDA 0 //GPIO PINS, NOT THE PIN NUMBERS!
#define I2C_SCL 1
#define I2C_SENSOR_ADDR 0x19 //ADDR

int main()
{
    stdio_init_all();

    DFRobot_AirQualitySensor *sensor = new DFRobot_AirQualitySensor(I2C_SENSOR_ADDR, I2C_SDA, I2C_SCL, I2C_PORT);
    
    while (1) {
        uint8_t vers = sensor->get_version();
        printf("Version is: %d\n", vers);

        uint16_t particle_num = sensor->gainParticleNum_Every0_1L(PARTICLENUM_0_3_UM_EVERY0_1L_AIR);
        printf("The number of particles with a diameter of 0.3um per 0.1 in lift-off is: %d\n", particle_num);
        uint16_t particle_concentration = sensor->gainParticleConcentration_mgm3(PARTICLE_PM1_0_STANDARD);
        printf("PM1.0 concentration is: %d mg/m3\n", particle_concentration);
        sleep_ms(1000);  // Wait 1 second between readings
    }

    return 0;
}
