#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"

#include "hardware/uart.h"

#include "hardware/i2c.h"
#include "DFRobot_AirQualitySensor.hpp"

#include "hardware/spi.h"
#include "radiolib/RadioLib.h"
#include "radiolib/hal/RPiPico/PicoHal.h"
#include "radiolib/Module.h"


#define BUFFER_SIZE 60

// I2C defines
#define I2C_PORT i2c1
#define I2C_SDA 2 //GPIO PINS, NOT THE PIN NUMBERS!
#define I2C_SCL 3
#define I2C_SENSOR_ADDR 0x19 //ADDR

// UART defines
#define UART_PORT_0 uart0
#define UART_PORT_1 uart1
#define BAUD_RATE 9600 //Common
#define UART_TX_PIN_CO 0 //CO sensor (0)
#define UART_RX_PIN_CO 1
#define UART_TX_PIN_SO2 4 //SO2 sensor (1)
#define UART_RX_PIN_SO2 5
#define UART_TX_PIN_NO2 12 //NO2 sensor (0)
#define UART_RX_PIN_NO2 13
#define UART_TX_PIN_O3 20 //O3 sensor (1)
#define UART_RX_PIN_O3 21

static int uart0_devices_txpins[2] = {UART_TX_PIN_CO, UART_TX_PIN_NO2};
static int uart0_devices_rxpins[2] = {UART_RX_PIN_CO, UART_RX_PIN_NO2};
static int uart1_devices_txpins[2] = {UART_TX_PIN_SO2, UART_TX_PIN_O3};
static int uart1_devices_rxpins[2] = {UART_RX_PIN_SO2, UART_RX_PIN_O3};

char uart0_buffer[BUFFER_SIZE];
char uart1_buffer[BUFFER_SIZE];
static volatile int buffer_head = 0;
static volatile int buffer_tail = 0;

bool msg0_ready = false;
bool msg1_ready = false;

// Handler for uart0 channel
void on_uart_rx_0() {
    while (uart_is_readable(UART_PORT_0)) {
        char c = uart_getc(UART_PORT_0);
        int next_head = (buffer_head + 1) % BUFFER_SIZE;

        if (next_head != buffer_tail) {  // Espacio disponible en el buffer
            uart0_buffer[buffer_head] = c;
            buffer_head = next_head;
        }

        if(c == '\r') {
            msg0_ready = true;
        }
    }

    
}

// Handler for uart1 channel
void on_uart_rx_1() {
    while (uart_is_readable(UART_PORT_1)) {
        char c = uart_getc(UART_PORT_1);
        int next_head = (buffer_head + 1) % BUFFER_SIZE;

        if (next_head != buffer_tail) {  // Espacio disponible en el buffer
            uart1_buffer[buffer_head] = c;
            buffer_head = next_head;
        }

        if(c == '\r') {
            msg1_ready = true;
        }
        
    }
    //buffer_head = 0; buffer_tail = 0;
    //printf("MENSAJE 1 INTERRUPCION: %s\n", uart1_buffer);
}

void configure_uart(uart_inst_t* uart_port, int tx_pin, int rx_pin)
{
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    uart_set_hw_flow(uart_port, false, false);
    uart_set_format(uart_port, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart_port, false); // Desactiva el FIFO

    //handler for uart port specified
    if(uart_port == UART_PORT_0) { irq_set_exclusive_handler(UART0_IRQ, on_uart_rx_0); irq_set_enabled(UART0_IRQ, true);} 
    else { irq_set_exclusive_handler(UART1_IRQ, on_uart_rx_1); irq_set_enabled(UART1_IRQ, true); }
    
    uart_set_irq_enables(uart_port, true, false);
}

void uart_flush_rx_buffer(uart_inst_t *uart_id) {
    while (uart_is_readable(uart_id)) {
        uart_getc(uart_id);  // Read and discard all characters
    }
}

void print_info(char *data)
{
    char *token;
    char *delim = ",";

    // Tokenize the string
    token = strtok(data, delim); 
    printf("Sensor SN: %s\n", token);

    token = strtok(NULL, delim);
    printf("PPB: %s\n", token);

    token = strtok(NULL, delim);
    printf("TEMPERATURE: %s\n", token);

    token = strtok(NULL, delim);
    printf("RELATIVE HUMIDITY: %s\n", token);

    token = strtok(NULL, delim);
    printf("ADC_G (GAS): %s\n", token);

    token = strtok(NULL, delim);
    printf("ADC_T (TEMPERATURE): %s\n", token);

    token = strtok(NULL, delim);
    printf("ADC_H (RELATIVE HUMIDITY): %s\n", token);
}

void make_pm2_5_reading()
{
    DFRobot_AirQualitySensor *sensor = new DFRobot_AirQualitySensor(I2C_SENSOR_ADDR, I2C_SDA, I2C_SCL, I2C_PORT);
    uint8_t vers = sensor->get_version();
    printf("Version is: %d\n", vers);

    uint16_t particle_num = sensor->gainParticleNum_Every0_1L(PARTICLENUM_0_3_UM_EVERY0_1L_AIR);
    printf("The number of particles with a diameter of 0.3um per 0.1 in lift-off is: %d\n", particle_num);
    uint16_t particle_concentration = sensor->gainParticleConcentration_mgm3(PARTICLE_PM1_0_STANDARD);
    printf("PM1.0 concentration is: %d mg/m3\n", particle_concentration);
    sleep_ms(1000);  // Wait 1 second between readings
}

void make_uart_readings()
{
    uart_init(UART_PORT_1, BAUD_RATE);
    uart_init(UART_PORT_0, BAUD_RATE);

    for (int i=0; i<10; i++)
    {

        sleep_ms(500);
        configure_uart(UART_PORT_1, uart1_devices_txpins[0], uart1_devices_rxpins[0]);
        printf("\n\nMENSAJE PUERTOS 4-5, UART1 (N02)\n\n");
        uart_putc(UART_PORT_1, '\r');
        sleep_ms(500);
        while(!msg1_ready);
        msg1_ready = false;
        buffer_head = 0;
        buffer_tail = 0;
        // printf("\n%s\n", uart1_buffer);
        // sleep_ms(1000);
        print_info(uart1_buffer);
        
        configure_uart(UART_PORT_0, uart0_devices_txpins[1], uart0_devices_rxpins[1]);
        uart_putc(UART_PORT_0, '\r');
        printf("\n\nMENSAJE PUERTOS 12-13, UART0 (CO)\n\n");
        sleep_ms(500);
        while(!msg0_ready);
        msg0_ready = false;
        buffer_head = 0;
        buffer_tail = 0;
        // printf("\n%s\n", uart0_buffer);
        // sleep_ms(1000);
        print_info(uart0_buffer);

        uart_flush_rx_buffer(UART_PORT_1);
        uart_deinit(UART_PORT_1);
        gpio_set_function(uart1_devices_txpins[0], GPIO_FUNC_NULL);
        gpio_set_function(uart1_devices_rxpins[0], GPIO_FUNC_NULL);
        irq_remove_handler(UART1_IRQ, on_uart_rx_1);
        
        uart_flush_rx_buffer(UART_PORT_0);
        uart_deinit(UART_PORT_0);
        gpio_set_function(uart0_devices_txpins[1], GPIO_FUNC_NULL);
        gpio_set_function(uart0_devices_rxpins[1], GPIO_FUNC_NULL);
        irq_remove_handler(UART0_IRQ, on_uart_rx_0);
        
        for (int i = 0; i < BUFFER_SIZE; i++) {
            uart0_buffer[i] = '\0';  // Set each element to the null character
            uart1_buffer[i] = '\0';
        }

        sleep_ms(500);
        uart_init(UART_PORT_1, BAUD_RATE);
        uart_init(UART_PORT_0, BAUD_RATE);
        sleep_ms(1000);

        configure_uart(UART_PORT_1, uart1_devices_txpins[1], uart1_devices_rxpins[1]);
        sleep_ms(500);
        printf("\n\nMENSAJE PUERTOS 20-21, UART1 (O3)\n\n");
        uart_putc(UART_PORT_1, '\r');
        sleep_ms(500);
        while(!msg1_ready);
        msg1_ready = false;
        buffer_head = 0;
        buffer_tail = 0;
        // printf("\n%s\n", uart1_buffer);
        // sleep_ms(1000);
        print_info(uart1_buffer);

        configure_uart(UART_PORT_0, uart0_devices_txpins[0], uart0_devices_rxpins[0]);
        sleep_ms(500);
        printf("\n\nMENSAJE PUERTOS 0-1, UART0 (S02)\n\n");
        uart_putc(UART_PORT_0, '\r');
        sleep_ms(500);
        while(!msg0_ready);
        msg0_ready = false;
        buffer_head = 0;
        buffer_tail = 0;
        // printf("\n%s\n", uart0_buffer);
        // sleep_ms(1000);
        print_info(uart0_buffer);

        uart_flush_rx_buffer(UART_PORT_1);
        uart_deinit(UART_PORT_1);
        gpio_set_function(uart1_devices_txpins[1], GPIO_FUNC_NULL);
        gpio_set_function(uart1_devices_rxpins[1], GPIO_FUNC_NULL);
        irq_remove_handler(UART1_IRQ, on_uart_rx_1);
        
        uart_flush_rx_buffer(UART_PORT_0);
        uart_deinit(UART_PORT_0);
        gpio_set_function(uart0_devices_txpins[0], GPIO_FUNC_NULL);
        gpio_set_function(uart0_devices_rxpins[0], GPIO_FUNC_NULL);
        irq_remove_handler(UART0_IRQ, on_uart_rx_0);
        
        for (int i = 0; i < BUFFER_SIZE; i++) {
            uart0_buffer[i] = '\0';  // Set each element to the null character
            uart1_buffer[i] = '\0';
        }

        sleep_ms(500);
        uart_init(UART_PORT_1, BAUD_RATE);
        uart_init(UART_PORT_0, BAUD_RATE);
        sleep_ms(1000);
    }

}

void use_lora_module()
{
    // SX1262 has the following connections:
    // NSS pin:   10
    // DIO1 pin:  2
    // NRST pin:  3
    // BUSY pin:  9

    // sx = SX1262(clk=Pin(18), 
    //              mosi=Pin(19), miso=Pin(16), 
    //              cs=Pin(8), rst=Pin(9), irq=Pin(7),
    //              gpio=Pin(15)
    // )

    // sx.begin(freq=868.3, bw=125, sf=10, cr=6, syncWord=0x12,
    //         power=-5, currentLimit=60.0, preambleLength=8,
    //         implicit=False, implicitLen=0xFF,
    //         crcOn=False, txIq=False, rxIq=True,
    //         tcxoVoltage=1.7, useRegulatorLDO=False, blocking=False
    // )

    // Initialize GPIO pins
    spi_init(spi_default, 1000 * 1000);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI);

    // Make the SPI pins available to picotool
    //bi_decl(bi_4pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI));

    // Initialize HAL
    PicoHal *pico2 = new PicoHal(spi_default, PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, 8000000);
    Module *sx1262_module = new Module(pico2, PICO_DEFAULT_SPI_CSN_PIN, 7, 28, 15);

}

int main()
{
    stdio_init_all();

    make_pm2_5_reading();
    make_uart_readings();
    use_lora_module();

    
    return 0;
}