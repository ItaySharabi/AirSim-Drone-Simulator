MAX_ROLL = 1.5  # roll angel
MIN_ROLL = -1.5

# A good set of P-D Constants!
KP = 8.0
KI = 0.0
KD = 196.0

# Best values so far
# KP = 4.0
# KI = 0.0
# KD = 150.0

# KP = 2.0
# KI = 0.0
# KD = 50.0

# KP = 2.0
# KI = 0.0
# KD = 100.0

LIDAR_ERR = 0.02


class RightWallPID:
    def __init__(self, target=None, P=KP, I=KI, D=KD):
        self.kp = P
        self.ki = I
        self.kd = D
        self.setpoint = target
        self.error = 0
        self.integral_error = 0
        self.error_last = 0
        self.derivative_error = 0
        self.output = 0
        self.i = 0
        self.output_table = []

    def compute(self, lidar_input, dt=1.0):

        if self.i == 1000000:
            self.i = 0

        # Compute the error term:
        self.error = lidar_input - self.setpoint
        self.integral_error = self.integral_error + (self.error * dt)

        if self.error != self.error_last:
            self.derivative_error = (self.error - self.error_last) / dt

        self.error_last = self.error

        self.output_table.append(
            (len(self.output_table),
             (self.kp * self.error),
             (self.ki * self.integral_error),
             (self.kd * self.derivative_error),
             self.output)
        )

        self.output = (self.kp * self.error) + (self.ki * self.integral_error) + (self.kd * self.derivative_error)

        if self.output > MAX_ROLL:
            self.output = MAX_ROLL
        if self.output < MIN_ROLL:
            self.output = MIN_ROLL
        self.i += 1
        return self.output

    def __repr__(self):
        return f'------------- PID -------------\n' \
               f'Target: {self.setpoint} ||| perr: {self.error}, ierr: {self.integral_error}, derr: {self.derivative_error}'
