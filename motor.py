import board
from adafruit_motor import stepper
from adafruit_motorkit import MotorKit

kit = MotorKit(i2c=board.I2C())

kit.stepper1.release()


try:
    print("Double coil steps")
    while True:
        kit.stepper1.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)

finally:
    print("releasing motor....exiting")
