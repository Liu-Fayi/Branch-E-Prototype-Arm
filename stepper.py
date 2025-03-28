import board
import digitalio
from digitalio import DigitalInOut
import time

class Stepper:
    def __init__(self, dir, step):
        '''input pins are the pin numbers of the stepper motor in form board.GPXX'''
        self.dir = DigitalInOut(dir)
        self.step = DigitalInOut(step)
        self.dir.direction = digitalio.Direction.OUTPUT
        self.step.direction = digitalio.Direction.OUTPUT
        self.DELAY = 0.001
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
            direction = 1
        else:
            direction = 0
        for i in range(steps):
            self.onestep(direction)
            # time.sleep(self.DELAY)
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
            direction = 1
        else:
            direction = 0
        self.onestep(direction)
        # time.sleep(self.DELAY)
        if forward:
            self.step_count += 1
        else:
            self.step_count -= 1

    def release(self):
        '''clears coils so no power is sent to motor & shaft can spin freely'''
        self.motor.release()
        
    def onestep(self, direction):
        '''input direction is the direction to turn'''
        if direction:
            self.dir.value = False
        else:
            self.dir.value = True
        self.step.value = not self.step.value
        time.sleep(self.DELAY)