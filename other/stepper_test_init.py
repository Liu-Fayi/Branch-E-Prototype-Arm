from actuating_pico.stepper import Stepper
import time
import board
import digitalio
import pwmio
import adafruit_motor.servo
from arm import Arm

arm = Arm()

arm.base_rotate.turn_to(10,True)
arm.base_calibrate()