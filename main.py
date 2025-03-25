from arm import Arm
from I2C import I2C
import time

X_OFF = 0
Y_OFF = 0

def main():
    arm = Arm()
    i2c = I2C()
    i2c.send_go_signal()
    arm.calibrate_base()
    arm.calibrate_z()

    x,y,_,angle = i2c.wait_for_data()

    arm.move_to(x+X_OFF, y + Y_OFF, 10, angle)
    arm.open_claw()
    time.sleep(1)
    arm.move_to(x+X_OFF, y + Y_OFF, 3, angle)
    arm.close_claw()
    time.sleep(1)
    arm.move_to(x+X_OFF, y + Y_OFF, 10, angle)
    arm.move_to(0, 0, 10, 0)
    arm.open_claw()

if __name__ == "__main__":
    main()

    

