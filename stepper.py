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

    def turn_to_target(self, target):
        '''input target is the target position to turn to'''
        steps = target - self.step_count
        if steps > 0:
            self.turn_to(steps, True)
        else:
            self.turn_to(-steps, False)

    def turn_to(self, steps, forward):
        '''input steps is the number of steps to turn and direction is the direction to turn'''
        if forward:
            direction = stepper.FORWARD
        else:
            direction = stepper.BACKWARD
        for i in range(steps):
            self.motor.onestep(direction=direction)
            time.sleep(self.DELAY)
            if forward:
                self.step_count += 1
            else:
                self.step_count -= 1
    
    def get_position(self):
        '''returns the current position of the stepper motor'''
        return self.step_count
    
    def reset_position(self):
        '''resets the position of the stepper motor to 0'''
        self.step_count = 0

    def turn(self, forward):
        '''input direction is the direction to turn'''
        if forward:
            direction = stepper.FORWARD
        else:
            direction = stepper.BACKWARD
        self.motor.onestep(direction=direction)
        time.sleep(self.DELAY)
        if forward:
            self.step_count += 1
        else:
            self.step_count -= 1

    def release(self):
        '''clears coils so no power is sent to motor & shaft can spin freely'''
        self.motor.release()
        