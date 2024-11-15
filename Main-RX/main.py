
from sx1262 import SX1262
from dfrobot_airqualitysensor import *
import utime
import struct
from machine import I2C, Pin, UART

PRESET = 0xFFFF
POLYNOMIAL = 0xA001 # Modbus

# Funcion callback para los envíos del módulo LoRa
def rx_callback(events):
    if events & SX1262.RX_DONE:
        
        data = lora.recv()
        bytes_payload = splitIntoChunks(data[0], 5) # type: ignore
        print(bytes_payload)
        
        crc = cbor_decode_integer(bytes_payload[0])
        data_airsensor1 = cbor_decode_float_single(bytes_payload[1])
        data_airsensor2 = cbor_decode_float_single(bytes_payload[2])
        data_no2_1 = cbor_decode_float_single(bytes_payload[3])
        data_no2_2 = cbor_decode_float_single(bytes_payload[4])
        data_no2_3 = cbor_decode_float_single(bytes_payload[5])
        timestamp = cbor_decode_integer(bytes_payload[6])

        payload = (crc, data_airsensor1, data_airsensor2, data_no2_1, data_no2_2, data_no2_3, timestamp)
        print(payload)

        #print(type(data)) # type: ignore
        #print(data) # type: ignore
        #print("Message received successfully")

        


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

def splitIntoChunks(byte_sequence, chunk_size):
    return [byte_sequence[i:i + chunk_size] for i in range(0, len(byte_sequence), chunk_size)]

# Function to decode CBOR bytes/hex
def cbor_decode_hex(data):
    """
    Decodes CBOR data that represents a byte string.
    Expects the input to start with a major type 2 (0x40).
    """
    initial_byte = data[0]
    if initial_byte < 0x40 or initial_byte > 0x57:
        raise ValueError("Invalid CBOR encoding for byte string")
    length = initial_byte - 0x40  # Length is stored in the low 5 bits
    return data[1:1 + length], data[1 + length:]  # Decoded byte string and remaining bytes


# Function to decode CBOR float32
def cbor_decode_float_single(data):
    """
    Decodes CBOR single-precision (float32) data.
    Expects the input to start with 0xFA.
    """
    if data[0] != 0xFA:
        raise ValueError("Invalid CBOR encoding for float32")
    float_bytes = data[1:5]  # Next 4 bytes are the float data
    value = struct.unpack('>f', float_bytes)[0]  # Big-endian
    return value, data[5:]  # Decoded float and remaining bytes


# Function to decode CBOR integers
def cbor_decode_integer(data):
    """
    Decodes CBOR integer data (both major type 0 for positive and type 1 for negative).
    """
    initial_byte = int(data[0])  # Convert first byte to an integer
    major_type = (initial_byte >> 5) & 0x07  # Extract the major type
    additional_info = initial_byte & 0x1F  # Extract the additional information

    if major_type not in [0, 1]:
        raise ValueError("Invalid CBOR encoding for integer")

    if additional_info < 24:
        # Integer is stored directly in the additional info
        value = additional_info
        consumed_bytes = 1
    elif additional_info == 24:
        # Followed by 1 byte
        value = int(data[1])
        consumed_bytes = 2
    elif additional_info == 25:
        # Followed by 2 bytes
        value = int.from_bytes(data[1:3], 'big')
        consumed_bytes = 3
    elif additional_info == 26:
        # Followed by 4 bytes
        value = int.from_bytes(data[1:5], 'big')
        consumed_bytes = 5
    elif additional_info == 27:
        # Followed by 8 bytes
        value = int.from_bytes(data[1:9], 'big')
        consumed_bytes = 9
    else:
        raise ValueError("Unsupported integer size")

    if major_type == 1:
        # Negative integer
        value = -1 - value

    return value, data[consumed_bytes:]  # Decoded integer and remaining bytes



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
         crcOn=False, txIq=False, rxIq=True,
         tcxoVoltage=1.7, useRegulatorLDO=False, blocking=False)


    # Envio no bloqueante y asociación con funcion callback
    sx.setBlockingCallback(False, rx_callback)
    
    return sx


print("\n--------------")
print("Modulo RX LoRa")
print("--------------\n")

lora = loadLora()

print(lora.getStatus())
lora.startReceiveDutyCycleAuto() #seteando DC de recepcion
data = lora.recv()

# Callback en recepcion
while 1:
    continue
    