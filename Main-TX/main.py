
from sx1262 import SX1262
from dfrobot_airqualitysensor import *
import utime
from machine import Pin, UART
import gc
    

def main():

    print("\n--------------")
    print("Modulo TX LoRa")
    print("--------------\n")
    
    tx_pins = [Pin(12, mode=Pin.ALT_UART), Pin(4, mode=Pin.ALT_UART), Pin(0, mode=Pin.ALT_UART), Pin(20, mode=Pin.ALT_UART), Pin(8, mode=Pin.ALT_UART)]
    rx_pins = [Pin(13, mode=Pin.ALT_UART), Pin(5, mode=Pin.ALT_UART), Pin(1, mode=Pin.ALT_UART), Pin(21, mode=Pin.ALT_UART), Pin(9, mode=Pin.ALT_UART)]
    b = b''

    for i in range(0,10):

        utime.sleep(3)
        uart1 = UART(1,baudrate=9600, tx=tx_pins[4], rx=rx_pins[4], bits=8, parity=None, stop=1, timeout=3000)
        utime.sleep(3)
        uart1.write(b'\x0D')
        b = uart1.read()

        print("SENSOR A")

        print(b)
        uart1.deinit()

        utime.sleep(3)
        # uart2 = UART(1, baudrate=9600, tx=tx_pins[3], rx=rx_pins[3], bits=8, parity=None, stop=1, timeout=2000)
        # utime.sleep(3)
        # uart2.write("\r")
        # b = uart2.read()

        # print("SENSOR B")

        # print(b)
        # uart2 = None

if __name__ == "__main__":
    main()