MAX_PITCH = -2.0  # pitch angel
MIN_PITCH = 2.0

# A good set of P-D Constants!

# KP = 10.0
# KI = 0.0
# KD = 0.0
KP = 4.0
KI = 0.0
KD = 150.0


class FrontPID:
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

    def compute(self, lidar_front):
        # right = 2.5
        # 2.5 - 1.5 = 1.0
        # 2.999 - 1.5 = 1.444444
        if self.i == 1000000:
            self.i = 0

        # Compute the error term:
        self.error = lidar_front - self.setpoint
        self.integral_error = self.integral_error + self.error

        if self.error != self.error_last:
            self.derivative_error = self.error - self.error_last

        self.error_last = self.error

        self.output_table.append(
            (len(self.output_table),
             (self.kp * self.error),
             (self.ki * self.integral_error),
             (self.kd * self.derivative_error),
             self.output)
        )

        self.output = ((self.kp * self.error) + (self.ki * self.integral_error) + (self.kd * self.derivative_error))
        if self.i % 10 == 0:
            print(f'FrontPID output: {self.output}')
        if self.output < MAX_PITCH:
            self.output = MAX_PITCH
        if self.output > MIN_PITCH:
            self.output = MIN_PITCH
        self.i += 1
        return self.output

    def __repr__(self):
        return f'------------- PID -------------\n' \
               f'Target: {self.setpoint} ||| perr: {self.error}, ierr: {self.integral_error}, derr: {self.derivative_error}'
