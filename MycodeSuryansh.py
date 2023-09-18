#!/usr/bin/env python3

from ev3dev.ev3 import *

# Connect the motors to the appropriate ports.
left_motor = LargeMotor('outB')
right_motor = LargeMotor('outC')

# Set the motor speed and time to create the lemniscate shape.
speed = 300  # You can adjust this value to control the speed of the robot.
duration = 10  # You can adjust this value to control the duration of movement.

# Function to move both motors forward.
def move_forward():
    left_motor.run_forever(speed_sp=speed)
    right_motor.run_forever(speed_sp=speed)

# Function to move both motors backward.
def move_backward():
    left_motor.run_forever(speed_sp=-speed)
    right_motor.run_forever(speed_sp=-speed)

# Function to stop both motors.
def stop_motors():
    left_motor.stop()
    right_motor.stop()

# Main code
try:
    while True:
        move_forward()
        time.sleep(duration)
        stop_motors()
        move_backward()
        time.sleep(duration)
        stop_motors()

except KeyboardInterrupt:
    stop_motors()
