# rpico-lora : LoRa/LoRaWAN + conjunto de sensores I2C/SPI/UART en Raspberry Pi Pico 2

## Desarrollo realizado

### (LEGACY) MicroPython:
Es necesario para poder arrancarlo, el archivo .uf2 que contiene la compilación de microPython para el RP2350 basado en RISC-V. Para ello hay que pulsar en la Pico2 el botón BOOTSEL a la vez que se conecta el dispositivo al ordenador. Después, copiar este archivo .uf2 al directorio raíz del dispositivo USB que saldrá. Esto reinicia a la Pico2 en modo MicroPython, y a partir de aquí ya se conectará a /dev/ttyACM0, según dmesg. Si dan errores, reiniciar y volver a intentarlo.

**PROBLEMA:** el componente UART es complejo de orquestar con más de dos sensores: por ello, el desarrollo de la aplicación se enfocará en el uso de C/C++.

### C/C++:
Es necesario para instalar los plugins de VSCode disponibles. Una vez realizado, se puede compilar y ejecutar en placa fácilmente. Por el momento, la interfaz I2C y el manejo de múltiples UART (4) está realizado correctamente. El código desarrollado puede verse en [este enlace](https://github.com/fluctlights/rpico-lora/blob/main/Pruebas/C/SensorI2C/sensor/sensor.cpp) . En ese código se recorren todos los sensores de UART que están conectados a la Raspberry Pi Pico 2, de forma que se realiza una lectura de cada uno de ellos. Para ello, cada vez que se quiera realizar una lectura sobre el mismo raíl UART pero diferentes pines, es necesario reconfigurar el raíl por completo. El programa está integrado en un bucle infinito, por lo que se repetirán las mismas acciones a lo largo del tiempo. 

Es posible ver información de estos sensores al comunicarse por puerto serie con minicom o similar. Un ejemplo de lectura de valores de un sensor sería el siguiente:

```
MENSAJE PUERTOS 12-13, UART0 (CO)

Sensor SN: 051424010627
PPB:  -3340
TEMPERATURE:  2080
RELATIVE HUMIDITY:  4723
ADC_G (GAS):  32248
ADC_T (TEMPERATURE):  25232
ADC_H (RELATIVE HUMIDITY):  27910 

```

1. Sensor SN: número de serie del módulo
2. PPB: partículas por billón detectadas, es la concentración de gas presente en el medio
3. ADC_G/H/T: otra forma de representar los valores de gas, temperatura y humedad (max 65535)

Actualmente, los sensores están conectados de la siguiente forma a la placa de desarrollo:

![](https://github.com/fluctlights/rpico-lora/blob/main/Assets/esquema_sensores_uart.jpg)


**Lista To-Do:**

- Confirmar el buen funcionamiento del módulo LoRa en C (RadioLib repo en C)
- Implementar un quinto sensor UART para utilizar red 4G, con comandos AT en C puro es posible.


## Documentación

- Micropython y su firmware: https://www.tomshardware.com/raspberry-pi/raspberry-pi-pico/how-to-install-micropython-for-risc-v-on-the-raspberry-pi-pico-2
- Modulo LoRa: https://www.waveshare.com/wiki/Core1262-868M
- Modulos de Gases:  https://www.spec-sensors.com/wp-content/uploads/2024/06/DGS2-970-Series-Datasheet-24a.pdf
- Calculadora LoRa: https://www.loratools.nl/#/airtime
- Modulo 4G: https://www.waveshare.com/wiki/SIM7600X_4G_Module#Raspberry_Pi_Raspbian_Internet_Access

## Código de refererencia

- Proyecto LoRa en C: https://github.com/jgromes/RadioLib
- Chip LoRa (SX1262): https://github.com/ehong-tl/micropySX126X/tree/master
- Chip LoRa (SX1262): https://github.com/GereZoltan/LoRaWAN/tree/main
- Sensor PM2.5: https://github.com/DFRobot/DFRobot_AirQualitySensor/tree/master
- Deepsleep Pi Pico 2: https://ghubcoder.github.io/posts/deep-sleeping-the-pico-micropython/
