
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


# def readAirQuality():
#     airquality_sensor.awake()
#     utime.sleep(5)
#     num_particles_bigger_than_3_um_per_0_1_l = airquality_sensor.gain_particlenum_every0_1l(airquality_sensor.PARTICLENUM_0_3_UM_EVERY0_1L_AIR)
#     print(f"The number of particles with a diameter of 0.3um per 0.1 in lift-off is: {num_particles_bigger_than_3_um_per_0_1_l}\n")
#     concentration_pm2_5_in_ug_m3 = airquality_sensor.gain_particle_concentration_ugm3(airquality_sensor.PARTICLE_PM2_5_STANDARD)
#     print(f"PM2.5 concentration is: {concentration_pm2_5_in_ug_m3} mg/m3 \n")
#     airquality_sensor.set_lowpower()
#     utime.sleep(3)
#     #los valores son integer, pero casteamos a float -> manejar la recepcion CON LO QUE TENGO AHORA, porque reviso el paquete en bloques de 5 en 5
#     return (float(num_particles_bigger_than_3_um_per_0_1_l), float(concentration_pm2_5_in_ug_m3)) 

#     # Si modifico la codificacion de integer para CBOR, soy capaz de retornar enteros en vez de floats -> FUNCIONA PERO NO SIEMPRE......
#     # return (num_particles_bigger_than_3_um_per_0_1_l, concentration_pm2_5_in_ug_m3)

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

# def readUartData(received_data):
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
#                 cs=Pin(28), rst=Pin(17), irq=Pin(7),
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

# def read_with_timeout(uart):

#     start_time = utime.ticks_ms()  # Record the start time
#     buffer = b""  # Initialize an empty buffer
    
#     while utime.ticks_diff(utime.ticks_ms(), start_time) < 2000:
#         if uart.any():  # Check if there is data available
#             data = uart.read(1)  # Read one byte at a time
#             if data:
#                 buffer += data
#         else:
#             utime.sleep(0.01)  # Avoid busy-waiting, sleep for 10ms

#     return buffer if buffer else None  # Return None if no data was read

# def makeSensorRead(uart):
#     # uart.write("r")
#     # utime.sleep_ms(1500)
#     # uart.write("Z")
#     # utime.sleep_ms(1500)
#     clear_uart_buffer(uart)

#     received_data = b''
#     uart.write("\r")
#     #utime.sleep_ms(75)
#     #received_data = read_with_timeout(uart)
#     data_sensor = readUartData(received_data)

#     #uart.write("s")
#     #clear_uart_buffer(uart)
#     return data_sensor

# def main():

#     print("\n--------------")
#     print("Modulo TX LoRa")
#     print("--------------\n")

#     global lora, airquality_sensor, sensors, start

#     airquality_sensor = DFRobot_AirQualitySensor()
#     airquality_sensor.awake()
#     utime.sleep(3)

#     version = airquality_sensor.gain_version()
#     print("Firmware version is: " + str(version))
#     data_airquality = readAirQuality()

#     # Inicio modulo lora (TX)
#     lora = loadLora()
#     while (lora.getDeviceErrors() != 0): # 0 es que no hay errores
#         lora = loadLora()


#     # Inicio railes UART
#     data_sensors = list()
#     # uart0 = UART(0, 9600)
#     # uart1 = UART(1, 9600)


#     tx_pins = [Pin(12), Pin(4), Pin(0), Pin(20), Pin(8)]
#     rx_pins = [Pin(13), Pin(5), Pin(1), Pin(21), Pin(9)]

#     moreSensorsToRead = True

    

#     # uart0 = UART(0, 9600, tx=tx_pins[0], rx=rx_pins[0], bits=8, parity=None, stop=1)
#     # utime.sleep(1)
#     # received_data = b''
#     # uart0.write(b'\x0d')
#     # utime.sleep(1)

#     # a = uart0.read()
#     # print(a)
#     # uart0.deinit()



#     # uart0 = UART(0, 9600, tx=tx_pins[2], rx=rx_pins[2], bits=8, parity=None, stop=1, timeout=2000)
#     # received_data = b''
#     # uart0.write(b'\x0d')
#     # a = uart0.read()
#     # print(a)
#     # uart0.deinit()

#     while(1):
#         # utime.sleep(5)
#         # uart0 = UART(0, 9600, tx=tx_pins[2], rx=rx_pins[2], bits=8, parity=None, stop=1)
#         # received_data = b''
#         # uart0.write("\r")
#         # a = uart0.read()
#         # print(a)
#         # uart0.deinit()
        

#         # FUNCIONA OK
#         # utime.sleep(5)
#         # uart0 = UART(1, 9600, tx=tx_pins[1], rx=rx_pins[1], bits=8, parity=None, stop=1)
#         # received_data = b''
#         # uart0.write("\r")
#         # a = uart0.read()
#         # print(a)
#         # uart0.deinit()
        
#         # utime.sleep(5)
#         # uart0 = UART(1, 9600, tx=tx_pins[3], rx=rx_pins[3], bits=8, parity=None, stop=1)
#         # received_data = b''
#         # uart0.write("\r")
#         # a = uart0.read()
#         # print(a)
#         # uart0.deinit()

#         # uart0 = UART(0, 9600, tx=tx_pins[0], rx=rx_pins[0], bits=8, parity=None, stop=1)
#         # received_data = b''
#         # uart0.write("\r")
#         # a = uart0.read()
#         # print(a)
#         # uart0.deinit()

#     #utime.sleep_ms(75)
#     #received_data = read_with_timeout(uart)
#     #readUartData(received_data)




#     # 10 mensajes de prueba
#     # for i in range(0, 1):

#     #     sensors_ptr = 0
#     #     moreSensorsToRead = True

#     #     while (moreSensorsToRead):

#     #         if (sensors_ptr != 4):
#     #             if((sensors_ptr % 2) == 0): # par, uart0
                    
#     #                 uart0.init(9600, tx=tx_pins[sensors_ptr], rx=rx_pins[sensors_ptr], bits=8, parity=None, stop=1)
#     #                 data_sensors.append(makeSensorRead(uart0))
#     #                 utime.sleep(1)

#     #             else: # impar
                    
#     #                 uart1.init(9600, tx=tx_pins[sensors_ptr], rx=rx_pins[sensors_ptr], bits=8, parity=None, stop=1)
#     #                 data_sensors.append(makeSensorRead(uart1))
#     #                 utime.sleep(1)
                
#     #             sensors_ptr+=1

#     #         else: # GSM

#     #             uart1.init(9600, tx=tx_pins[sensors_ptr], rx=rx_pins[sensors_ptr], bits=8, parity=None, stop=1)
#     #             utime.sleep_ms(1500)
#     #             uart1.write("at")
#     #             utime.sleep_ms(100)
#     #             received_data = b''
#     #             received_data = read_with_timeout(uart1)
#     #             print(received_data)
#     #             moreSensorsToRead = False

    
#         # sensors = list()
#         # # Timestamp del mensaje (variable global)
#         # start = utime.time() 
#         # # Concatenacion y procesado de las tuplas de valores
#         # data = data_airquality + data_sensors[0] + data_sensors[3]
       
#         # data_sensors.clear()
#         # payload_info = data + (start,)
#         # checksum = crc16(payload_info)
#         # payload_data = (checksum,) + payload_info
#         # payload = processPayload(payload_data)


#         # Envio
#         #lora.send(payload) # duty cycle incorporado
    
#         print(lora.getStatus()) # activado, y cuando se produzca callback se pone en sleep
