
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
        print('Message sent!')
        lora.sleep()

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
def cbor_encode_float_single(value):
    cbor_type = b'\xFA'  # Single-precision
    float_bytes = struct.pack('>f', value)  # Big-endian
    return cbor_type + float_bytes

# Funcion para codificacion de int en CBOR
def cbor_encode_integer(major_type, value):
    if value < 24:
        # Fits into 5 bits of the initial byte
        return bytes([(major_type << 5) | value])
    elif value < 256:
        # Additional 8-bit integer follows
        return bytes([(major_type << 5) | 24, value])
    elif value < 65536:
        # Additional 16-bit integer follows
        return bytes([(major_type << 5) | 25]) + value.to_bytes(2, 'big')
    elif value < 4294967296:
        # Additional 32-bit integer follows
        return bytes([(major_type << 5) | 26]) + value.to_bytes(4, 'big')
    elif value < 18446744073709551616:
        # Additional 64-bit integer follows
        return bytes([(major_type << 5) | 27]) + value.to_bytes(8, 'big')
    else:
        raise ValueError("Value too large for CBOR encoding")

# Creacion del payload en formato CBOR
def processPayload(payload):
    data = bytes()
    fields = 0
    sizes = []

    for element in payload:
        if(type(element) == int):
            data += cbor_encode_integer(0, element)
            sizes.append(len(cbor_encode_integer(0, element)))

        else:
            data += cbor_encode_float_single(element)
            sizes.append(len(cbor_encode_float_single(element)))

        fields+=1

        print("Fields %d", fields)
        print(sizes)
    return data

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

############################
# SENSOR DIOXIDO NITROGENO #
############################

def loadNitrogenDioxide():

    # Inicio UART 0
    no2_sensor = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1), bits=8, parity=None, stop=1)

    # Activando sensor
    no2_sensor.write("r")
    utime.sleep_ms(1500)
    no2_sensor.write("Z")
    utime.sleep_ms(1500)

    # Activar modo una sola lectura para leer datos
    no2_sensor.write("\r")
    utime.sleep_ms(200)

    # Espera activa
    while no2_sensor.any() == None:
        continue

    # Leer dato
    received_data = no2_sensor.read()
    
    # Deepsleep (para reactivar hay que mandar cualquier caracter)
    no2_sensor.write("s")
    utime.sleep_ms(100)

    # Procesar datos y return
    received_data = received_data.decode('utf-8')
    received_data = received_data.split(',', 1)[1] # elimino primer elemento
    clean_data = [e.strip() for e in received_data.split(',')]

    vals = [float(e) for e in clean_data]
    if vals[0] < 0:
        vals[0] = 0.00

    # Temp (pos. 2) y Humedad (pos. 3) SON PORCENTAJES! LOS METO COMO INT PARA AHORRAR ESPACIO!
    gas_values = (vals[1], vals[2])
    # NO2 resultado (pos. ) (conversion del ADC) --> Float
    gas_max_adc = 65535
    gas_value = (vals[3]/gas_max_adc)*100

    # Concatenando
    values = gas_values + (gas_value,)

    return values

######################################
# EXTRAER NUMEROS LECTURA SENSOR SO2 #
######################################

def extractNumbers(data):

    numbers = []
    current_number = ''
    for char in data:
        if char.isdigit():
            current_number += char 
        elif current_number:
            numbers.append(current_number) 
            current_number = ''  
    if current_number: 
        numbers.append(current_number)
    return numbers

#########################
# SENSOR DIOXIDO AZUFRE #
#########################

def loadAzufreDioxide():

    # Inicio UART 1
    so2_sensor = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5), bits=8, parity=None, stop=1)

    # Activando sensor
    so2_sensor.write("r")
    utime.sleep_ms(1500)
    so2_sensor.write("Z")
    utime.sleep_ms(1500)

    # Activar modo una sola lectura para leer datos
    so2_sensor.write("\r")
    utime.sleep_ms(200)

    # Espera activa
    while so2_sensor.any() == None:
        continue

    # Leer dato
    received_data = so2_sensor.read()
    
    # Deepsleep (para reactivar hay que mandar cualquier caracter)
    so2_sensor.write("s")
    utime.sleep_ms(100)

    # Procesar datos y return
    decoded_data = ''.join(chr(b) if 32 <= b <= 126 or b in {10, 13} else '?' for b in received_data)
    clean_data = extractNumbers(decoded_data)
    clean_data = clean_data[1:] # elimino primer elemento

    vals = [float(e) for e in clean_data]
    if vals[0] < 0:
        vals[0] = 0.00

    # Temp (pos. 2) y Humedad (pos. 3) SON PORCENTAJES! LOS METO COMO INT PARA AHORRAR ESPACIO!
    gas_values = (vals[1], vals[2])
    # SO2 resultado (pos. ) (conversion del ADC) --> Float
    gas_max_adc = 65535
    gas_value = (vals[3]/gas_max_adc)*100

    # Concatenando
    values = gas_values + (gas_value,)
    return values


###############
# MODULO LORA #
###############

def loadLora():
    
    # Iniciar módulo LoRa (SPI)
    sx = SX1262(clk=Pin(18), 
                mosi=Pin(19), miso=Pin(16), 
                cs=Pin(8), rst=Pin(9), irq=Pin(7),
                gpio=Pin(15)
                )

    sx.begin(freq=868.3, bw=125, sf=10, cr=6, syncWord=0x12,
         power=-5, currentLimit=60.0, preambleLength=8,
         implicit=False, implicitLen=0xFF,
         crcOn=False, txIq=True, rxIq=False,
         tcxoVoltage=1.7, useRegulatorLDO=False, blocking=False)

    # Envio no bloqueante y asociación con funcion callback
    sx.setBlockingCallback(False, tx_callback)
    
    return sx

def main():

    print("\n--------------")
    print("Modulo TX LoRa")
    print("--------------\n")
    
    global start, lora

    # Inicio modulo lora (TX)
    lora = loadLora()
    while (lora.getDeviceErrors() != 0): # 0 es que no hay errores
        lora = loadLora()

    while 1:
        # Inicio de los sensores (faltan todavia)
        # airsensor, val1, val2 = loadAirquality()
        data_airsensor = (11.111, 11.222)
        data_no2 = loadNitrogenDioxide()
        data_so2 = loadAzufreDioxide()

        # Timestamp del mensaje (variable global)
        start = utime.time() 

        # Concatenacion y procesado de las tuplas de valores
        data = data_airsensor + data_no2 + data_so2
        payload_info = data + (start,)
        checksum = crc16(payload_info)
        payload_data = (checksum,) + payload_info
        payload = processPayload(payload_data)
        
        # Envio
        lora.send(payload) # duty cycle incorporado
        print(lora.getStatus()) # activado, y cuando se produzca callback se pone en sleep
        utime.sleep_us(100)

if __name__ == "__main__":
    main()