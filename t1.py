import board
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33
import adafruit_lis3mdl
import neopixel
import time

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

import math
### Sensor fusion using Euler angles
class EulerAngles:
    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    def update(self, accel, gyro, mag, dt):
        # Normalize the accelerometer and magnetometer measurements
        norm = math.sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2])
        accel = [accel[i] / norm for i in range(3)]
        norm = math.sqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2])
        mag = [mag[i] / norm for i in range(3)]

        # Compute roll, pitch and yaw using the accelerometer and magnetometer data
        roll = math.atan2(accel[1], accel[2])
        pitch = math.asin(-accel[0])
        yaw = math.atan2(mag[1] * math.cos(pitch) - mag[2] * math.sin(pitch),
                         mag[0] * math.cos(roll) + mag[1] * math.sin(roll) * math.sin(pitch) + mag[2] * math.sin(roll) * math.cos(pitch))

        # Integrate the angular velocity from the gyroscope
        self.roll += gyro[0] * dt
        self.pitch += gyro[1] * dt
        self.yaw += gyro[2] * dt

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
    euler_angles = EulerAngles()
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
            accel = sensor.acceleration
            gyro = sensor.gyro
            mag = magnetometer.magnetic
            s_2 = time.time()

            dt = s_2 - s
            euler_angles.update(accel, gyro, mag, dt)

            print("Roll: ", euler_angles.roll)
            print("Pitch: ", euler_angles.pitch)
            print("Yaw: ", euler_angles.yaw)
            #print(sensor.acceleration)
            #print(sensor.gyro)
            #print(magnetometer.magnetic)
            #dat = a_x,a_y,a_z,g_x,g_y,g_z,m_x,m_y,m_z
            #print(dat)
            print("Roll: ", euler_angles.roll)
            print("Pitch: ", euler_angles.pitch)
            print("Yaw: ", euler_angles.yaw)
            #uart.write(str(dat).encode())