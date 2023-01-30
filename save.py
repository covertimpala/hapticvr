import time
import math
import board
import busio
import adafruit_lsm9ds1

# Initialize the I2C bus and LSM9DS1 sensor
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

# Set the gain for the accelerometer and gyroscope
sensor.accelerometer_range = adafruit_lsm9ds1.Range.G2
sensor.gyroscope_range = adafruit_lsm9ds1.Range.DPS500

# Set the gain for the magnetometer
sensor.magnetometer_range = adafruit_lsm9ds1.MagnetometerRange.GAUSS4

# Set the sample rate for the accelerometer and gyroscope
sensor.accelerometer_rate = adafruit_lsm9ds1.AccelerometerRate.RATE_104_HZ
sensor.gyroscope_rate = adafruit_lsm9ds1.GyroscopeRate.RATE_104_HZ

# Set the sample rate for the magnetometer
sensor.magnetometer_rate = adafruit_lsm9ds1.MagnetometerRate.RATE_100_HZ

# Complementary filter coefficients
alpha = 0.98

# Initialize the angles
pitch = 0
roll = 0

while True:
    # Read the accelerometer values
    ax, ay, az = sensor.acceleration

    # Read the gyroscope values
    gx, gy, gz = sensor.gyro

    # Calculate the angular velocity
    angular_velocity = math.sqrt(gx**2 + gy**2 + gz**2)

    # Calculate the pitch and roll angles from the accelerometer values
    pitch = math.atan2(ay, az)
    roll = math.atan2(-ax, math.sqrt(ay**2 + az**2))

    # Fuse the pitch and roll angles from the accelerometer and gyroscope
    pitch = alpha * (pitch + gy * dt) + (1 - alpha) * pitch
    roll = alpha * (roll + gx * dt) + (1 - alpha) * roll

    # Read the magnetometer values
    mx, my, mz = sensor.magnetic

    # Do something with the fused values
    print("Pitch: ", pitch, "Roll: ", roll, "Magnetometer: ", (mx, my, mz))

    time.sleep(0.01)
