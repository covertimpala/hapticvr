import math

class MadgwickFilter:

    def __init__(self, sample_period, beta): #good starting value for beta is 0.1
        self.sample_period = sample_period
        self.beta = beta
        self.q = [1, 0, 0, 0]

    def update(self, accel, gyro, mag):
        q = self.q

        # Normalise accelerometer measurement
        if not (accel[0] == 0 and accel[1] == 0 and accel[2] == 0):
            norm = math.sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2])
            accel = [accel[i] / norm for i in range(3)]

        # Normalise magnetometer measurement
        if not (mag[0] == 0 and mag[1] == 0 and mag[2] == 0):
            norm = math.sqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2])
            mag = [mag[i] / norm for i in range(3)]

        # Reference direction of Earth's magnetic field
        h = [0, 0, 0]
        h[0] = mag[0] * q[0] + mag[1] * q[1] + mag[2] * q[2]
        h[1] = mag[0] * q[3] - mag[1] * q[2] + mag[2] * q[1]
        h[2] = mag[0] * q[2] + mag[1] * q[3] - mag[2] * q[0]

        # Gradient decent algorithm corrective step
        f = [0, 0, 0]
        f[0] = 2 * (q[1] * q[3] - q[0] * q[2]) - accel[0]
        f[1] = 2 * (q[0] * q[1] + q[2] * q[3]) - accel[1]
        f[2] = 2 * (0.5 - q[1] * q[1] - q[2] * q[2]) - accel[2]

        J_T = [-2 * q[2], 2 * q[3], -2 * q[0], 2 * q[1]]
        J = [2 * q[1], 2 * q[2], 2 * q[3], 2 * q[0]]

        # check the length of gyro list 
        print(gyro)
        gyro = gyro + [1] * (4 - len(gyro))
        f = f + [1] * (4 - len(f))
        

        # Apply feedback terms
        gyro = [gyro[i] - self.beta * f[i] for i in range(4)]
        q_dot = [-0.5 * J[i] * gyro[i] for i in range(4)]

        # Integrate rate of change of quaternion
        q = [q[i] + q_dot[i] * self.sample_period for i in range(4)]

        # Normalise quaternion
        norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
        self.q = [q[i] / norm for i in range(4)]

        global roll, pitch, yaw
        roll = math.atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]*q[1] + q[2]*q[2]))
        pitch = math.asin(2*(q[0]*q[2] - q[3]*q[1]))
        yaw = math.atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]*q[2] + q[3]*q[3]))

sample_period = 0.01 #adjust according to rate of measurements
beta = 0.1
m_filter = MadgwickFilter(sample_period, beta)
accel = [0,0,9.81]
gyro = [1,1,1] #4th part scalar

mag = [7,8,9]
m_filter.update(accel, gyro, mag)
print(roll, pitch, yaw)