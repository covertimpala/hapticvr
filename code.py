import board
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33
import adafruit_lis3mdl
import neopixel
import time
import math
#import busio

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

# Complementary filter coefficients
alpha = 0.98

# Initialize the angles
pitch = 0
roll = 0


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
            ax, ay, az = sensor.acceleration
            gx, gy, gz = sensor.gyro
            mx, my, mz = magnetometer.magnetic
            s_2 = time.time()
            angular_velocity = math.sqrt(gx**2 + gy**2 + gz**2)
            pitch = math.atan2(ay, az)
            roll = math.atan2(-ax, math.sqrt(ay**2 + az**2))
            dt = s_2 - s
            pitch = alpha * (pitch + gy * dt) + (1 - alpha) * pitch
            roll = alpha * (roll + gx * dt) + (1 - alpha) * roll
            #print(sensor.acceleration)
            #print(sensor.gyro)
            #print(magnetometer.magnetic)
            #dat = a_x,a_y,a_z,g_x,g_y,g_z,m_x,m_y,m_z
            print("Pitch: ", pitch, "Roll: ", roll, "Magnetometer: ", (mx, my, mz))
            time.sleep(0.01)
            #uart.write(str(ang).encode())