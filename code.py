import board
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33
import adafruit_lis3mdl
import neopixel
import time

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

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
    while not ble.connected:
        pass

    print("connected")

    while ble.connected:
        if uart.in_waiting:
            f = uart.readline().decode("utf-8")
            print(f)
            a_x, a_y, a_z = sensor.acceleration
            g_x, g_y, g_z = sensor.gyro
            m_x, m_y, m_z = magnetometer.magnetic
            #print(sensor.acceleration)
            #print(sensor.gyro)
            #print(magnetometer.magnetic)
            dat = a_x,a_y,a_z,g_x,g_y,g_z,m_x,m_y,m_z
            print(dat)
            uart.write(str.encode(dat))