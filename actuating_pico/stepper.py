import board
import digitalio
from digitalio import DigitalInOut
import time

class Stepper:
    def __init__(self, dir, step, sleep):
        '''Initializes the stepper motor controller with the given control pins.
        
        Args:
            dir: The direction control pin 
            step: The step control pin
            sleep: The sleep control pin to enable/disable motor power
        '''
        # Configure the direction, step, and sleep pins as digital outputs.
        self.dir = DigitalInOut(dir)
        self.step = DigitalInOut(step)
        self.sleep = DigitalInOut(sleep)
        self.dir.direction = digitalio.Direction.OUTPUT
        self.step.direction = digitalio.Direction.OUTPUT
        self.sleep.direction = digitalio.Direction.OUTPUT
        # Wake up the motor by default.
        self.sleep.value = True
        self.DELAY = 0.001     # Default delay between steps (controls motor speed)
        self.step_count = 0    # Counter to track the current step position
        
    def sleep(self):
        # Puts the stepper motor into sleep mode by disabling power.
        self.sleep.value = False
        
    def wake_up(self):
        # Wakes the stepper motor up by enabling power.
        self.sleep.value = True

    def turn_to_target(self, target):
        '''Moves the motor to an absolute target position.
        
        Args:
            target: The desired target step count.
        '''
        steps = target - self.step_count   # Calculate the number of steps required.
        if steps > 0:
            self.turn_to(steps, True)      # Move forward if positive steps are needed.
        else:
            self.turn_to(-steps, False)    # Move in reverse if negative steps are needed.
    
    def turn_to_target_vel(self, target, vel):
        '''Moves the motor to a target position at a specified velocity.
        
        Args:
            target: The target step count.
            vel: The desired velocity (steps per second).
        '''
        temp = self.DELAY         # Save the current delay value.
        self.DELAY = 1/vel        # Adjust the delay to match the desired velocity.
        self.turn_to_target(target)
        self.DELAY = temp         # Restore the original delay.
    
    def turn_to(self, steps, forward):
        '''Turns the motor a specified number of steps in a given direction.
        
        Args:
            steps: Number of steps to move.
            forward: Boolean indicating direction (True for forward, False for reverse).
        '''
        # Determine step direction.
        if forward:
            direction = 1
        else:
            direction = 0
        # Execute each step.
        for i in range(steps):
            self.onestep(direction)
            # Update the step counter based on the direction.
            if forward:
                self.step_count += 1
            else:
                self.step_count -= 1
    
    def turn_to_vel(self, steps, forward, vel):
        '''Turns the motor a specified number of steps at a given velocity.
        
        Args:
            steps: Number of steps to move.
            forward: Boolean indicating direction.
            vel: The desired velocity (steps per second).
        '''
        temp = self.DELAY       # Store current delay.
        self.DELAY = 1/vel      # Set DELAY to control speed.
        self.turn_to(steps, forward)
        self.DELAY = temp       # Restore original delay.
    
    def set_velocity(self, vel):
        '''Sets the motor's speed.
        
        Args:
            vel: Desired speed in steps per second.
        '''
        self.DELAY = 1/vel
    
    def get_velocity(self):
        '''Returns the current velocity (steps per second).'''
        return 1/self.DELAY
    
    def get_position(self):
        '''Returns the current step count (position) of the motor.'''
        return self.step_count
    
    def get_frequency(self):
        '''Returns the stepping frequency in Hz.'''
        return 1/self.DELAY
    
    def reset_position(self):
        '''Resets the step counter to 0.'''
        self.step_count = 0
    
    def turn(self, forward):
        '''Takes one step in the specified direction.
        
        Args:
            forward: Boolean indicating direction.
        '''
        if forward:
            direction = 1
        else:
            direction = 0
        self.onestep(direction)
        # Update the position counter.
        if forward:
            self.step_count += 1
        else:
            self.step_count -= 1
    
    def turn_vel(self, forward, vel):
        '''Takes one step at a specified velocity.
        
        Args:
            forward: Boolean indicating direction.
            vel: Desired velocity in steps per second.
        '''
        temp = self.DELAY    # Save the current delay.
        self.DELAY = 1/vel   # Adjust delay for the desired velocity.
        self.turn(forward)
        self.DELAY = temp    # Restore original delay.
    
    def release(self):
        '''Clears motor coils so that no power is applied, allowing the shaft to spin freely.'''
        self.motor.release() # Note: self.motor should be defined or implemented elsewhere.
    
    def onestep(self, direction):
        '''Triggers one step of the motor in the specified direction. Should not be called directly, instead, use methods turn() or turn_to().
        
        Args:
            direction: Numeric flag for direction (non-zero for one direction, zero for the other).
        '''
        # Set the direction pin based on the given value.
        if direction:
            self.dir.value = False
        else:
            self.dir.value = True
        # Toggle the step pin to initiate a step.
        self.step.value = not self.step.value
        # Introduce a delay between steps to control stepping frequency.
        time.sleep(self.DELAY)