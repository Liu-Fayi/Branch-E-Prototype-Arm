import board
import pwmio
from adafruit_motor import servo
import time

class Servo:
    def __init__(self, pin, pwm_freq, pwm_duty_cycle):
        '''input pin is the pin number of the servo in form board.GPXX'''
        self.pwm = pwmio.PWMOut(pin, frequency=pwm_freq, duty_cycle=pwm_duty_cycle)
        self.servo = servo.Servo(self.pwm)
        self.angle = 0
        self.servo.angle = self.angle
        self.DELAY = 0.05

    def set_angle(self, angle):
        '''input angle is the angle to set the servo to'''
        self.angle = angle
        self.servo.angle = self.angle
        time.sleep(self.DELAY)

    def get_angle(self):
        '''returns the current angle of the servo'''
        return self.angle