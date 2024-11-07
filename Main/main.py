# -*- coding: utf-8 -*

from sx1262 import SX1262
from dfrobot_airqualitysensor import *
import utime

# Funcion callback para los envíos del módulo LoRa
def tx_callback(events):
    if events & SX1262.TX_DONE:
        print('Done')
        print(utime.time()-start)

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
    # Iniciar módulo LoRa
    lora = SX1262(spi_bus=1, clk=10, 
                mosi=11, miso=12, 
                cs=3, irq=20, 
                rst=15, gpio=2
                )
    
    # Parametros validos en Europa
    lora.begin(freq=868, bw=125.0, sf=10, cr=8, syncWord=0x12,
            power=14, currentLimit=60.0, preambleLength=8,
            implicit=False, implicitLen=0xFF,
            crcOn=False, txIq=False, rxIq=False,
            tcxoVoltage=1.7, useRegulatorLDO=False, 
            blocking=True 
            )

    # Envio no bloqueante y asociación con funcion callback
    lora.setBlockingCallback(False, tx_callback)
    return lora


print("AAAAAAAA")
# Inicio de los componentes
# airsensor, val1, val2 = loadAirquality() 
# lora = loadLora()

global start
start = utime.time() # Tiempo de inicio (global)

# TODO :: IMPLEMENTAR TX/RX
#lora.send(str((val1, val2)))
print("Message sent successfully")
