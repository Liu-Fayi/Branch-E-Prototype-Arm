from actuating_pico.stepper import Stepper
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
# arm.calibrate()


arm.elbow_move(math.radians(0))
ut.send_go_signal()
print("scaning")
print("scaning")
x,y,z,angle = ut.wait_for_data()
print(math.degrees(angle))
arm.calibrate()
# for i in range(5):
arm.open_claw()
arm.move_to(x, y, 1 , angle)
# arm.move_to(180, 50, 10 , math.radians(50))
arm.close_claw()
arm.collect_branch()
# arm.move_to(x, y, 1 , angle)
# arm.elbow_move(1.57)
# for i in range(0,190,10):
#     arm.wrist_move(math.radians(i))
#     print(i)
#     time.sleep(1)

# time.sleep(5)


# print(x,y,z,angle)
# while True:
    # arm.open_claw()
    # arm.wrist_move(0)
    # arm.elbow_move(1.57)
    #print("open")
    # time.sleep(1)
    # arm.close_claw()
#print("close")
    # arm.wrist_move(math.radians(43))
    # arm.elbow_move(math.radians(105))
    # time.sleep(1)

#arm.wrist_move(0)
#time.sleep(2)



