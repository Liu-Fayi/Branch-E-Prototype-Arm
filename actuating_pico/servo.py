import board
import pwmio
from adafruit_motor import servo
import time

class Servo:
    def __init__(self, pin, pwm_freq, pwm_duty_cycle):
        '''
        Initializes the servo motor controller.
        
        Args:
            pin: The pin number for the servo (e.g., board.GPXX)
            pwm_freq: Frequency (in Hz) for the PWM signal
            pwm_duty_cycle: Duty cycle for the PWM signal initialization
        '''
        # Create a PWM output on the given pin with specified frequency and duty cycle.
        self.pwm = pwmio.PWMOut(pin, frequency=pwm_freq, duty_cycle=pwm_duty_cycle)
        # Initialize the servo object using the PWM signal.
        self.servo = servo.Servo(self.pwm)
        self.angle = 0          # Current angle of the servo, initially set to 0 to avoid undefined variables
        self.DELAY = 0.05       # Delay in seconds after setting angle for smooth movement

    def set_angle(self, angle):
        '''
        Sets the servo to the specified angle.
        
        Args:
            angle: The target angle in degrees
        '''
        self.angle = angle                  # Update internal angle state
        self.servo.angle = self.angle       # Command the servo to go to the target angle
        time.sleep(self.DELAY)              # Wait briefly to allow the servo to move

    def get_angle(self):
        '''
        Returns the current angle of the servo.
        
        Returns:
            The current angle (in degrees) stored in the Servo object
        '''
        return self.angle