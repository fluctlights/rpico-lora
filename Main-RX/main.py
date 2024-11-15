
from sx1262 import SX1262
from dfrobot_airqualitysensor import *
import utime
import struct
from machine import I2C, Pin, UART

PRESET = 0xFFFF
POLYNOMIAL = 0xA001 # Modbus

# Funcion callback para los envíos del módulo LoRa
def rx_callback():
    print("GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG")
    if SX1262.RX_DONE:
        print('Done')
        print(utime.time())


# Funcion para el CRC
def crc16(data):
    crc = PRESET
    for byte in data:
        crc = crc ^ int(byte)
        for _ in range(8):
            if (crc & 1) == 0:
                crc = (crc >> 1) ^ POLYNOMIAL
            else:
                crc >>= 1
                
    return crc ^ PRESET

# Funcion para codificacion de bytes/hex en CBOR
def cbor_encode_hex(data):
    length = len(data)
    return bytes([0x40 + length]) + data  # bytes major type (0x40) + data

# Funcion para codificacion de float32 en CBOR
def encode_float_single(value):
    cbor_type = b'\xFA'  # Single-precision
    float_bytes = struct.pack('>f', value)  # Big-endian
    return cbor_type + float_bytes

###########################
# SENSOR CALIDAD DEL AIRE #
###########################

def loadAirquality():
    # Inicio I2C
    i2c = I2C(1, scl=Pin(27), sda=Pin(26), freq=100000)
    I2C_ADDRESS = 0x19 # Direccion del sensor

    # Inicio y recogida de datos del sensor de calidad del aire (PM2.5)
    airquality = DFRobot_AirQualitySensor(i2c, I2C_ADDRESS)

    version = sensor.gain_version()
    print("Firmware version is: " + str(version))
    num_particles_bigger_than_2_5_um_per_0_1_l = airquality.gain_particlenum_every0_1l(airquality.PARTICLENUM_2_5_UM_EVERY0_1L_AIR)
    concentration_pm2_5_in_ug_m3 = airquality.gain_particle_concentration_ugm3(airquality.PARTICLE_PM2_5_STANDARD)
    return airquality, num_particles_bigger_than_2_5_um_per_0_1_l, concentration_pm2_5_in_ug_m3



###############
# MODULO LORA #
###############

def loadLora():

    #spi = SPI(0, baudrate=1000000, polarity=0, phase=0, sck=Pin(18), mosi=Pin(19), miso=Pin(16))
    
    # Iniciar módulo LoRa
    sx = SX1262(clk=Pin(18), 
                mosi=Pin(19), miso=Pin(16), 
                cs=Pin(8), rst=Pin(9), irq=Pin(7),
                gpio=Pin(15)
                )

    sx.begin(freq=868.3, bw=125, sf=10, cr=6, syncWord=0x12,
         power=-5, currentLimit=60.0, preambleLength=8,
         implicit=False, implicitLen=0xFF,
         crcOn=True, txIq=True, rxIq=True,
         tcxoVoltage=1.7, useRegulatorLDO=False, blocking=True)


    # Envio no bloqueante y asociación con funcion callback
    #sx.setBlockingCallback(False, callback=rx_callback)
    
    return sx


print("\n--------------")
print("Modulo RX LoRa")
print("--------------\n")

lora = loadLora()

print(lora.getStatus())
lora.startReceiveDutyCycleAuto()
data = lora.recv()
print(data)
print(lora.getStatus())
print(lora.getDeviceErrors()) #0 es que no hay errores
print("Message sent successfully")