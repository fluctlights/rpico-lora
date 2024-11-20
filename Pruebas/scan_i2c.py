from machine import Pin, I2C
import time

# Initialize I2C bus (Pins 21=SDA, 22=SCL on default Pico setup)
i2c = I2C(0, scl=Pin(22), sda=Pin(21))

print("Scanning I2C bus...")
devices = i2c.scan()

if devices:
    print("I2C devices found:")
    for device in devices:
        print(f" - Device found at address: {hex(device)}")
else:
    print("No I2C devices found.")
