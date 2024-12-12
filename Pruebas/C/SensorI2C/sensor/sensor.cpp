#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "DFRobot_AirQualitySensor.hpp"

#define BUFSIZE 1024

// I2C defines
#define I2C_PORT i2c1
#define I2C_SDA 2 //GPIO PINS, NOT THE PIN NUMBERS!
#define I2C_SCL 3
#define I2C_SENSOR_ADDR 0x19 //ADDR

// UART defines
#define UART_PORT_0 uart0
#define UART_PORT_1 uart1
#define BAUD_RATE 9600 //Common
#define UART_TX_PIN_CO 0 //CO sensor
#define UART_RX_PIN_CO 1
#define UART_TX_PIN_SO2 4 //SO2 sensor
#define UART_RX_PIN_SO2 5
#define UART_TX_PIN_NO2 12 //NO2 sensor
#define UART_RX_PIN_NO2 13
#define UART_TX_PIN_O3 20 //O3 sensor
#define UART_RX_PIN_O3 21

static int uart0_devices_pins[4] = {UART_TX_PIN_CO, UART_TX_PIN_CO, UART_TX_PIN_NO2, UART_RX_PIN_NO2};
static int uart1_devices_pins[4] = {UART_TX_PIN_SO2, UART_TX_PIN_SO2, UART_TX_PIN_O3, UART_RX_PIN_O3};
char ch_0[BUFSIZE];
char ch_1[BUFSIZE];
char msg0[BUFSIZE];
char msg1[BUFSIZE];

// Handler for uart0 channel
void on_uart_rx_0() {
    memset(ch_0, 0, BUFSIZE);
    int ch0_index = 0;
    while (uart_is_readable(UART_PORT_0)) {
        ch_0[ch0_index++] = uart_getc(UART_PORT_0);
    }

    for(int i=0; i<BUFSIZE; i++)
    {
        msg0[i] = ch_0[i];
    }
    
}

// Handler for uart1 channel
void on_uart_rx_1() {
    memset(ch_1, 0, BUFSIZE);
    int ch1_index = 0;
    while (uart_is_readable(UART_PORT_1)) {
        ch_1[ch1_index++] = uart_getc(UART_PORT_1);
    }
    for(int i=0; i<BUFSIZE; i++)
    {
        msg1[i] = ch_0[i];
    }
}

void configure_uart(uart_inst_t* uart_port, int tx_pin, int rx_pin)
{
    
    gpio_set_function(tx_pin, UART_FUNCSEL_NUM(&uart_port, tx_pin));
    gpio_set_function(rx_pin, UART_FUNCSEL_NUM(&uart_port, rx_pin));
    int __unused actual = uart_set_baudrate(uart_port, BAUD_RATE);
    uart_set_hw_flow(uart_port, false, false);
    uart_set_format(uart_port, 8, 1, UART_PARITY_NONE);

    //handler for uart port specified
    if(uart_port == UART_PORT_0) { irq_set_exclusive_handler(UART0_IRQ, on_uart_rx_0); irq_set_enabled(UART0_IRQ, true);} 
    else { irq_set_exclusive_handler(UART1_IRQ, on_uart_rx_1); irq_set_enabled(UART1_IRQ, true); }
    
    uart_set_irq_enables(uart_port, true, false);
}

int main()
{
    stdio_init_all();

    DFRobot_AirQualitySensor *sensor = new DFRobot_AirQualitySensor(I2C_SENSOR_ADDR, I2C_SDA, I2C_SCL, I2C_PORT);

    uart_init(UART_PORT_0, BAUD_RATE);
    uart_init(UART_PORT_1, BAUD_RATE);

    while (1) {
        uint8_t vers = sensor->get_version();
        printf("Version is: %d\n", vers);

        uint16_t particle_num = sensor->gainParticleNum_Every0_1L(PARTICLENUM_0_3_UM_EVERY0_1L_AIR);
        printf("The number of particles with a diameter of 0.3um per 0.1 in lift-off is: %d\n", particle_num);
        uint16_t particle_concentration = sensor->gainParticleConcentration_mgm3(PARTICLE_PM1_0_STANDARD);
        printf("PM1.0 concentration is: %d mg/m3\n", particle_concentration);
        sleep_ms(1000);  // Wait 1 second between readings
    
        for(int i=0; i<4; i+=2)
        {
            configure_uart(UART_PORT_0, uart0_devices_pins[i], uart0_devices_pins[i+1]);
            uart_puts(UART_PORT_0, "r");
            sleep_ms(1500);
            uart_puts(UART_PORT_0, "Z");
            sleep_ms(1500);
            
            uart_puts(UART_PORT_0, "\r");
            sleep_ms(50);
            for(int i=0; i<BUFSIZE; i++) {printf("%c", msg0[i]);}
            printf("\n");
        }

        for(int i=0; i<BUFSIZE; i++)
        {
            configure_uart(UART_PORT_1, uart1_devices_pins[i], uart1_devices_pins[i+1]);
            uart_puts(UART_PORT_0, "r");
            sleep_ms(1500);
            uart_puts(UART_PORT_0, "Z");
            sleep_ms(1500);

            uart_puts(UART_PORT_1, "\r");
            sleep_ms(50);
            for(int i=0; i<BUFSIZE; i++) {printf("%c", msg1[i]);}
            printf("\n");
        }


    }

    return 0;
}
