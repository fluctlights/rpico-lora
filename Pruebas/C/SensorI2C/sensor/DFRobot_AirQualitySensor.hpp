
/*!
 * Refactorizacion en C/C++ puro
 */

#ifndef __DFR0bot_AIR_H__
#define __DFR0bot_AIR_H__

#include <stdint.h>
#include <stdio.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"




// Open this macro to see the program running in detail
//#define ENABLE_DBG

#ifdef ENABLE_DBG
#define DBG(...)                     \
    {                                \
        Serial.print("[");           \
        Serial.print(__FUNCTION__);  \
        Serial.print("(): ");        \
        Serial.print(__LINE__);      \
        Serial.print(" ] 0x");         \
        Serial.println(__VA_ARGS__,HEX); \
    }
#else
#define DBG(...)
#endif

#define PARTICLE_PM1_0_STANDARD   0X05
#define PARTICLE_PM2_5_STANDARD   0X07
#define PARTICLE_PM10_STANDARD    0X09
#define PARTICLE_PM1_0_ATMOSPHERE 0X0B
#define PARTICLE_PM2_5_ATMOSPHERE 0X0D
#define PARTICLE_PM10_ATMOSPHERE  0X0F

#define PARTICLENUM_0_3_UM_EVERY0_1L_AIR 0X11
#define PARTICLENUM_0_5_UM_EVERY0_1L_AIR 0X13
#define PARTICLENUM_1_0_UM_EVERY0_1L_AIR 0X15
#define PARTICLENUM_2_5_UM_EVERY0_1L_AIR 0X17
#define PARTICLENUM_5_0_UM_EVERY0_1L_AIR 0X19
#define PARTICLENUM_10_UM_EVERY0_1L_AIR  0X1B

#define SENSOR_STATE 0X01
#define SENSOR_VERSION 0X1D

class DFRobot_AirQualitySensor {
public:
    DFRobot_AirQualitySensor(uint8_t i2c_addr, uint8_t i2c_sda, uint8_t i2c_scl, i2c_inst_t* i2c_port);
    ~DFRobot_AirQualitySensor(void){};

    uint16_t gainParticleConcentration_mgm3(uint8_t type);
    uint16_t gainParticleNum_Every0_1L(uint8_t type);
    uint8_t get_version();
    void lowpower();
    void awake();

protected:
    void write_register(uint8_t reg, const uint8_t *data, uint8_t len);
    int16_t read_register(uint8_t reg, uint8_t *data, uint8_t len);

private:
    uint8_t _i2c_addr;
    uint8_t _i2c_sda;
    uint8_t _i2c_scl;
    i2c_inst_t _i2c_port;
    void init_i2c();
    void look_for_addr();
    bool reserved_addr(uint8_t addr);
};

#endif
