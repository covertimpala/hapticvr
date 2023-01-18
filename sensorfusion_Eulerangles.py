import math

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
euler_angles = EulerAngles()
dt = .01 #time interval between sensor measurements
accel = [0.1, 0.2, 0.3]
gyro = [0.01, 0.02, 0.03]
mag = [0.1, 0.2, 0.3]
euler_angles.update(accel, gyro, mag, dt)

print("Roll: ", euler_angles.roll)
print("Pitch: ", euler_angles.pitch)
print("Yaw: ", euler_angles.yaw)