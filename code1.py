import board
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33
import adafruit_lis3mdl
import neopixel
import time

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

#neopixel-setup
pixels = neopixel.NeoPixel(board.NEOPIXEL, 1)
pixels[0] = (10, 0, 0)
#sensor setup
i2c = board.I2C()
sensor = LSM6DS33(i2c)
magnetometer = adafruit_lis3mdl.LIS3MDL(i2c)

ble = BLERadio()
ble.name = "Haptic-right"
uart = UARTService()
advertisement = ProvideServicesAdvertisement(uart)



while True:
    ble.start_advertising(advertisement)
    print("ready to connect")
    pixels[0] = (0, 5, 0)
    while not ble.connected:
        pass

    print("connected")
    pixels[0] = (5, 0, 2)

    while ble.connected:
            #m = uart.readline()
        #if m != "":
            #print(m.decode())
            #uart.write(m)
        #if uart.in_waiting:
         #   f = uart.readline().decode("utf-8")
          #  print(f)
            s = time.time()
            a_x, a_y, a_z = sensor.acceleration
            g_x, g_y, g_z = sensor.gyro
            m_x, m_y, m_z = magnetometer.magnetic
            s_2 = time.time()
            a_x = round(a_x,2)
            a_y = round(a_y,2)
            a_z = round(a_z,2)
            g_x = round(g_x,2)
            g_y = round(g_y,2)
            g_z = round(g_z,2)
            m_x = round(m_x,2)
            m_y = round(m_y,2)
            m_z = round(m_z,2)
            #print(sensor.acceleration)
            #print(sensor.gyro)
            #print(magnetometer.magnetic)
            dat = a_x,a_y,a_z,g_x,g_y,g_z,m_x,m_y,m_z
            print(dat)
            uart.write(str(dat).encode())