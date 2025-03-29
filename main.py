from stepper import Stepper
import time
import board
import digitalio
import pwmio
import adafruit_motor.servo
from arm import Arm
from uart import UARTRx
import math

ut = UARTRx()

arm = Arm()
arm.open_claw()

#arm.elbow_move(math.radians(70))
#arm.calibrate_z()
#arm.z_move_to(20)
#arm.z_move_to(1)
#arm.close_claw()
#time.sleep(1)
#arm.z_move_to(30)
# arm.calibrate_base()
#arm.elbow_move(math.radians(10))
#time.sleep(1)
#arm.open_claw()
#time.sleep(2)
# arm.base_rotate_to(math.radians(90))
# arm.sleep_steppers()
# arm.elbow_move(3.14/2)
# arm.wrist_move(1.57)
# arm.calibrate_base()
# arm.calibrate_z()
# arm.elbow_move(0)
#arm.open_claw()
#arm.wrist_move(0)
# arm.base_rotate.sleep()
# arm.z_movement.sleep()
# time.sleep(10)
# time.sleep(10)
# time.sleep(10)

ut.send_go_signal()
print("scaning")
print("scaning")
x,y,z,angle = ut.wait_for_data()
print(math.degrees(angle))
arm.calibrate()
arm.open_claw()
arm.move_to(x, y, 1 , angle)
arm.close_claw()
arm.collect_branch()


# print(x,y,z,angle)
#while True: 
    #arm.open_claw()
#arm.wrist_move(0)
    #arm.elbow_move(3.14)
    #print("open")
    #time.sleep(1)
    #arm.close_claw()
#print("close")
#arm.wrist_move(math.radians(43))
    #arm.elbow_move(2.4)
    #time.sleep(1)
    
#arm.wrist_move(0)
#time.sleep(2)



