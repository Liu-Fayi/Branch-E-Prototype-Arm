import board
import digitalio
from adafruit_motor import stepper
import time

class Stepper:
    def __init__(self, AIN1, AIN2, BIN1, BIN2):
        '''input pins are the pin numbers of the stepper motor in form board.GPXX'''
        self.coils = (digitalio.DigitalInOut(AIN1), digitalio.DigitalInOut(AIN2), digitalio.DigitalInOut(BIN1), digitalio.DigitalInOut(BIN2))
        for coil in self.coils:
            coil.direction = digitalio.Direction.OUTPUT
        self.motor = stepper.StepperMotor(self.coils[0], self.coils[1], self.coils[2], self.coils[3], microsteps=None)
        self.DELAY = 0.01
        self.step_count = 0

    def turn_to(self, steps, direction):
        '''input steps is the number of steps to turn and direction is the direction to turn'''
        for i in range(steps):
            self.motor.onestep(direction=direction)
            time.sleep(self.DELAY)
            self.step_count += 1
    
    def get_position(self):
        '''returns the current position of the stepper motor'''
        return self.step_count
    
    def reset_position(self):
        '''resets the position of the stepper motor to 0'''
        self.step_count = 0

    def turn(self, direction):
        '''input direction is the direction to turn'''
        self.motor.onestep(direction=direction)
        time.sleep(self.DELAY)
        self.step_count += 1

    def release(self):
        '''clears coils so no power is sent to motor & shaft can spin freely'''
        self.motor.release()
        