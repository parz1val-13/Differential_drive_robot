#!/usr/bin/env python3

from ev3dev2.motor import LargeMotor
from time import sleep

# Connect the motor to the appropriate port.
motor = LargeMotor('outB')

# Set the initial speed and increment value.
initial_speed = 0  # Starting speed (0% power)
max_speed = 50     # Maximum speed (50% power)
increment = 5      # Speed increment (5% power)
delay = 1           # Delay between speed increments (1 second)

try:
    while initial_speed <= max_speed:
        motor.on(initial_speed)  # Set the motor speed
        print(f"Motor Speed: {initial_speed}%")
        sleep(delay)
        initial_speed += increment

    # Once the maximum speed is reached, hold it for a while.
    motor.on(max_speed)
    sleep(5)  # Hold for 5 seconds

    # Gradually decrease the speed back to 0.
    while initial_speed >= 0:
        motor.on(initial_speed)
        print(f"Motor Speed: {initial_speed}%")
        sleep(delay)
        initial_speed -= increment

    # Turn off the motor.
    motor.off()

except KeyboardInterrupt:
    motor.off()



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



#!/usr/bin/env pybricks-micropython
from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor,TouchSensor,ColorSensor,InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port,Stop,Direction,Button,Color,SoundFile,ImageFile,Align) from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase
1
# Initialize two motors with default settings on Port B and Port C.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
# setup wheel diameter and axle_track
wheel_diameter = 56
axle_track = 114
# setup DriveBase
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
# This turns 90 deg/sec and moves 250 mm/sec 1 second
robot.drive_time(250, 90, 1000)
# This stops the motor and brakes for accuracy
robot.stop(Stop.BRAKE)

make a c
this is new starting point
make a c opposite directiom
new starting point`
make a c again same Direction
new starting point`
make a c again opposite Direction
repeat`

while (3):
    code for making a C
    stop
    code for making opposite C
    stop
    code for making a C
    stop
    code for making opposite C
    stop




#!/usr/bin/env python3

from ev3dev2.motor import OUTPUT_B, OUTPUT_C, LargeMotor
from math import pi
from time import sleep

# Connect the left and right motors to the appropriate ports.
left_motor = LargeMotor(OUTPUT_B)
right_motor = LargeMotor(OUTPUT_C)

# Define the circle parameters.
radius = 5  # Radius of the circle in centimeters
angular_speed = 90  # Angular speed in degrees per second

# Calculate the circumference of the circle.
circumference = 2 * pi * radius

# Calculate the time it takes to complete one full circle.
circle_time = circumference / angular_speed

try:
    while True:
        # Move both motors at the same speed in opposite directions to make a circle.
        left_motor.on_for_seconds(angular_speed, circle_time)
        right_motor.on_for_seconds(-angular_speed, circle_time)

except KeyboardInterrupt:
    # Stop the motors if the program is interrupted.
    left_motor.off()
    right_motor.off()
