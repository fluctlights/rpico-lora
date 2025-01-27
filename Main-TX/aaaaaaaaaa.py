


# from sx1262 import SX1262
# from dfrobot_airqualitysensor import *
# import utime
# import struct
# from machine import Pin, UART

# PRESET = 0xFFFF
# POLYNOMIAL = 0xA001 # Modbus
# I2C_ADDRESS = 0x19 # Dirección modulo calidad del aire

# start_time = utime.ticks_ms()  # Get the current time
# timeout_ms = 1000  # 1-second timeout

# # Funcion callback para los envíos del módulo LoRa
# def tx_callback(events):
#     if events & SX1262.TX_DONE:
#         print('Message sent!')
#         lora.sleep()

# # Funcion para el CRC
# def crc16(data):
#     crc = PRESET
#     for byte in data:
#         crc = crc ^ int(byte)
#         for _ in range(8):
#             if (crc & 1) == 0:
#                 crc = (crc >> 1) ^ POLYNOMIAL
#             else:
#                 crc >>= 1

#     return crc ^ PRESET

# # Funcion para codificacion de bytes/hex en CBOR
# def cbor_encode_hex(data):
#     length = len(data)
#     return bytes([0x40 + length]) + data  # bytes major type (0x40) + data

# # Funcion para codificacion de float32 en CBOR
# def cbor_encode_float_single(value):
#     cbor_type = b'\xFA'  # Single-precision
#     float_bytes = struct.pack('>f', value)  # Big-endian
#     return cbor_type + float_bytes

# # Funcion para codificacion de int en CBOR
# def cbor_encode_integer(major_type, value):
#     if value < 24:
#         # Fits into 5 bits of the initial byte
#         return bytes([(major_type << 5) | value])
#     elif value < 256:
#         # Additional 8-bit integer follows
#         return bytes([(major_type << 5) | 24, value])
#     elif value < 65536:
#         # Additional 16-bit integer follows
#         return bytes([(major_type << 5) | 25]) + value.to_bytes(2, 'big')
#     elif value < 4294967296:
#         # Additional 32-bit integer follows
#         return bytes([(major_type << 5) | 26]) + value.to_bytes(4, 'big')
#     elif value < 18446744073709551616:
#         # Additional 64-bit integer follows
#         return bytes([(major_type << 5) | 27]) + value.to_bytes(8, 'big')
#     else:
#         raise ValueError("Value too large for CBOR encoding")

# # PRUEBAS, NO TERMINA DE FUNCIONAR

# # def cbor_encode_integer(major_type, value):
# #     if value < 24:
# #         # Fits into 5 bits of the initial byte
# #         return bytes([(major_type << 5) | value]) + value.to_bytes(4, 'big') # not necessary THE ADDITION, but ideal for 5 byte number
# #     elif value < 256:
# #         # Additional 8-bit integer follows
# #         return bytes([(major_type << 5) | 24]) + value.to_bytes(4, 'big') # not necessary THE ADDITION, but ideal for 5 byte number ()
# #     elif value < 65536:
# #         # Additional 16-bit integer follows
# #         return bytes([(major_type << 5) | 25]) + value.to_bytes(4, 'big') # for 5 byte number (IT WAS 2! ORIGINALLY)
# #     elif value < 4294967296:
# #         # Additional 32-bit integer follows
# #         return bytes([(major_type << 5) | 26]) + value.to_bytes(4, 'big')
# #     elif value < 18446744073709551616:
# #         # Additional 64-bit integer follows
# #         return bytes([(major_type << 5) | 27]) + value.to_bytes(8, 'big')
# #     else:
# #         raise ValueError("Value too large for CBOR encoding")


# # Creacion del payload en formato CBOR
# def processPayload(payload):
#     data = bytes()
#     fields = 0
#     sizes = []

#     for element in payload:
#         if(type(element) == int):
#             data += cbor_encode_integer(0, element)
#             sizes.append(len(cbor_encode_integer(0, element)))

#         else:
#             data += cbor_encode_float_single(element)
#             sizes.append(len(cbor_encode_float_single(element)))

#         fields+=1

#     #print(f"Fields: {fields}")
#     #print(f"Sizes: {sizes}")
#     return data

# ###########################
# # SENSOR CALIDAD DEL AIRE #
# ###########################

# ############################
# # SENSOR DIOXIDO NITROGENO #
# ############################

# def readFirstWay(received_data):

#     received_data = received_data.split(',', 1)[1] # elimino primer elemento
#     clean_data = [e.strip() for e in received_data.split(',')]

#     vals = [float(e) for e in clean_data]
#     if vals[0] < 0:
#         vals[0] = 0.00

#     # Temp (pos. 2) y Humedad (pos. 3) SON PORCENTAJES! LOS METO COMO INT PARA AHORRAR ESPACIO!
#     gas_values = (vals[1], vals[2])
#     # NO2 resultado (pos. ) (conversion del ADC) --> Float
#     gas_max_adc = 65535
#     gas_value = (vals[3]/gas_max_adc)*100

#     # Concatenando
#     values = gas_values + (gas_value,)
#     print(values)
#     return values

# ###########################
# # EXTRAER NUMEROS LECTURA #
# ###########################

# def extractNumbers(data):

#     numbers = []
#     current_number = ''
#     for char in data:
#         if char.isdigit():
#             current_number += char 
#         elif current_number:
#             numbers.append(current_number) 
#             current_number = ''  
#     if current_number: 
#         numbers.append(current_number)
#     return numbers

# def readAlternativeWay(received_data):
#     # Procesar datos y return
#     decoded_data = ''.join(chr(b) if 32 <= b <= 126 or b in {10, 13} else '?' for b in received_data)
#     clean_data = extractNumbers(decoded_data)
#     clean_data = clean_data[1:] # elimino primer elemento
#     vals = [float(e) for e in clean_data]
#     # if vals[0] < 0:
#     #     vals[0] = 0.00

#     # Temp (pos. 2) y Humedad (pos. 3) SON PORCENTAJES! LOS METO COMO INT PARA AHORRAR ESPACIO!
#     gas_values = (vals[1], vals[2])
#     # SO2 resultado (pos. ) (conversion del ADC) --> Float
#     gas_max_adc = 65535
#     gas_value = (vals[3]/gas_max_adc)*100

#     # Concatenando
#     values = gas_values + (gas_value,)
#     print(values)
#     return values

# def clear_uart_buffer(uart):
#     while 1:
#         if uart.any() == None:  # Check if there's data in the RX buffer
#             break
#         else:
#             uart.read() # Read and discard data
#             break


# ###############
# # MODULO LORA #
# ###############

# def loadLora():

#     # Iniciar módulo LoRa (SPI)
#     sx = SX1262(clk=Pin(18), 
#                 mosi=Pin(19), miso=Pin(16), 
#                 cs=Pin(8), rst=Pin(9), irq=Pin(7),
#                 gpio=Pin(15)
#                 )

#     sx.begin(freq=868.3, bw=125, sf=10, cr=6, syncWord=0x12,
#          power=-5, currentLimit=60.0, preambleLength=8,
#          implicit=False, implicitLen=0xFF,
#          crcOn=False, txIq=True, rxIq=False,
#          tcxoVoltage=1.7, useRegulatorLDO=False, blocking=False)

#     # Envio no bloqueante y asociación con funcion callback
#     sx.setBlockingCallback(False, tx_callback)

#     return sx

# def main():

#     print("\n--------------")
#     print("Modulo TX LoRa")
#     print("--------------\n")

#     global start, lora, airquality_sensor, sensors

#     # Inicio modulo lora (TX)
#     lora = loadLora()
#     while (lora.getDeviceErrors() != 0): # 0 es que no hay errores
#         lora = loadLora()

#     # Inicio de sensores SPEC
#     sensors = list()
#     #no2_sensor = UART(0, baudrate=9600, tx=Pin(12), rx=Pin(13), bits=8, parity=None, stop=1)
#     #so2_sensor = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5), bits=8, parity=None, stop=1)
#     #o3_sensor = UART(1, baudrate=9600, tx=Pin(20), rx=Pin(21), bits=8, parity=None, stop=1)
#     #co_sensor = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1), bits=8, parity=None, stop=1)
#     #sensors.append(no2_sensor)
#     #sensors.append(o3_sensor)
#     #sensors.append(so2_sensor)
#     #sensors.append(co_sensor)

#     # while 1:
#     #     #data_airquality = readAirQuality()

#     #     for s in sensors:
#     #         s.write("r")
#     #         utime.sleep_ms(5000)
#     #         s.write("Z")
#     #         utime.sleep_ms(2000)

#     #         while 1:
#     #             if s.read() == None:
#     #                 break

#     #     for s in sensors:

#     #         utime.sleep(1)
#     #         received_data = b''
#     #         s.write("\r")
            
#     #         utime.sleep_ms(300)

#     #         while 1:
#     #             a = s.read()
#     #             if a == None:
#     #                 break
#     #             received_data += a
            
#     #         print(received_data)
#     #         info = readAlternativeWay(received_data)

#     #     for s in sensors:
#     #         utime.sleep_ms(100)
#     #         received_data = s.read()
#     #         print(received_data)
#     #         info = readAlternativeWay(received_data)
            
#     #         #info = readFirstWay(received_data)
#     #         utime.sleep(1)
            

#     data_no2 = (2,2)
#     utime.sleep(1)
#     # data_o3 = readAlternativeWay(sensors[2])
#     utime.sleep(1)
#     data_so2 = (2,2)
#     # utime.sleep(1)
#     # data_co = readAlternativeWay(sensors[3])

#     # Timestamp del mensaje (variable global)
#     start = utime.time() 

#     # Concatenacion y procesado de las tuplas de valores
#     data = (2,2) + data_no2 + data_so2
#     payload_info = data + (start,)
#     checksum = crc16(payload_info)
#     payload_data = (checksum,) + payload_info
#     payload = processPayload(payload_data)

#     # Envio
#     lora.send(payload) # duty cycle incorporado
#     # print(lora.getStatus()) # activado, y cuando se produzca callback se pone en sleep

#     while 1:
#         continue

# if __name__ == "__main__":
#     main()


