# rpico-lora : LoRa/LoRaWAN en Raspberry Pi Pico 2

## Raspberry Pi Pico 2 con MicroPython:
Es necesario para poder arrancarlo, el archivo .uf2 que contiene la compilación de microPython para el RP2350 basado en RISC-V. Para ello hay que pulsar en la Pico2 el botón BOOTSEL a la vez que se conecta el dispositivo al ordenador. Después, copiar este archivo .uf2 al directorio raíz del dispositivo USB que saldrá. Esto reinicia a la Pico2 en modo MicroPython, y a partir de aquí ya se conectará a /dev/ttyACM0, según dmesg. Si dan errores, reiniciar y volver a intentarlo.

Referencia: https://www.tomshardware.com/raspberry-pi/raspberry-pi-pico/how-to-install-micropython-for-risc-v-on-the-raspberry-pi-pico-2

## Librerías externas utilizadas

- Chip LoRa (SX1262): https://github.com/ehong-tl/micropySX126X/tree/master
- Sensor PM2.5: https://github.com/DFRobot/DFRobot_AirQualitySensor/tree/master
- Deepsleep Pi Pico 2: https://ghubcoder.github.io/posts/deep-sleeping-the-pico-micropython/

## IDE:
Se ha utilizado VSCode para este desarrollo, instalando las extensiones MicroPico y Raspberry Pi Pico, ambas dependientes entre sí. Esto proporciona un entorno similar a PlatformIO.

## Documentación

- Modulo LoRa: https://www.spec-sensors.com/wp-content/uploads/2024/06/DGS2-970-Series-Datasheet-24a.pdf
- Modulos de Gases: https://www.waveshare.com/wiki/Core1262-868M
- Calculadora LoRa: https://www.loratools.nl/#/airtime

