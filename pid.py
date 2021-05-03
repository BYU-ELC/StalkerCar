
class PID:
    def __init__(self, kP, kI, kD, setP, sT):
        self.kp = kP
        self.ki = kI
        self.kd = kD
        self.setPoint = setP
        self.sampleTime = sT
        self.pidOn = 1
        self.lastInput = 0
        self.iTerm = 0

    def setGains(self, kP, kI, kD):
        self.kp = kP
        self.ki = kI
        self.kd = kD

    def setSetpoint(self, setP):
        self.setPoint = setP

    def setSampleTime(self, sampleT):
        self.sampleTime = sampleT

    def pidToggle(self, toggle):
        self.pidOn = toggle

    def setBounds(self, maxLim, minLim):
        self.max = maxLim
        self.min = minLim

    def compute(self, inp):
        if self.pidOn:
            error = self.setPoint - inp
            self.iTerm += self.ki * error * self.sampleTime
            if self.iTerm > self.max:
                self.iTerm = self.max
            elif self.iTerm < self.min:
                self.iTerm = self.min
            dInput = (inp - self.lastInput)/self.sampleTime
            self.output = (self.kp * error) + self.iTerm - (self.kd * dInput)
            if self.output > self.max:
                self.output = self.max
            elif self.output < self.min:
                self.output = self.min
            self.lastInput = inp
            return self.output
        return False
