
from sx1262 import SX1262
from dfrobot_airqualitysensor import *
import utime
import struct
from machine import I2C, Pin, UART

PRESET = 0xFFFF
POLYNOMIAL = 0xA001 # Modbus

# Funcion callback para los envíos del módulo LoRa
def tx_callback(events):
    if events & SX1262.TX_DONE:
        print('Done')
        print(utime.time()-start)

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
         crcOn=True, txIq=False, rxIq=False,
         tcxoVoltage=1.7, useRegulatorLDO=False, blocking=True)


    # Envio no bloqueante y asociación con funcion callback
    sx.setBlockingCallback(False, tx_callback)
    
    return sx

############################
# SENSOR DIOXIDO NITROGENO #
############################

def loadNitrogenDioxide():

    # Inicio UART 0
    no2_sensor = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1), bits=8, parity=None, stop=1)

    # Activando sensor
    no2_sensor.write("r")
    utime.sleep_ms(1500)

    # Activar modo una sola lectura para leer datos
    no2_sensor.write("\r")

    # Espera activa
    while not no2_sensor.any():
        continue

    # Leer dato
    received_data = no2_sensor.read()
    
    # Deepsleep (para reactivar hay que mandar cualquier caracter)
    no2_sensor.write("s")

    # Procesar datos y return
    received_data = received_data.decode('utf-8')
    received_data = received_data.split(',', 1)[1] # elimino primer elemento
    clean_data = [e.strip() for e in received_data.split(',')]
    vals = [int(e) for e in clean_data]

    # Temp (pos. 2) y Humedad (pos. 3) SON PORCENTAJES! LOS METO COMO INT PARA AHORRAR ESPACIO!
    gas_values = (vals[1], vals[2])
    # NO2 resultado (pos. ) (conversion del ADC) --> Float
    gas_max_adc = 65535
    gas_value = (vals[3]/gas_max_adc)*100

    # Concatenando
    values = gas_values + (gas_value,)

    return values

print("\n--------------")
print("Modulo TX LoRa")
print("--------------\n")

# Inicio de los sensores (faltan todavia)
# airsensor, val1, val2 = loadAirquality()
data_airsensor = (11.111, 11.222)
data_no2 = loadNitrogenDioxide()

# Tiempo de inicio (global)
global start
start = utime.time() 

# Concatenacion de las tuplas de valores
data = data_airsensor + data_no2
payload_data = data + (start,)
checksum = crc16(payload_data)

# Inicio modulo lora (TX)
lora = loadLora()
print(lora.getStatus())
lora.send(b'aaaa')
print(lora.getStatus())
print(lora.getDeviceErrors()) #0 es que no hay errores
print("Message sent successfully")
print(payload_data)