#!/usr/bin/env python3

# An EV3 Python (library v2) solution to Exercise 3

# of the official Lego Robot Educator lessons that

# are part of the EV3 education software

from ev3dev2.motor import MoveTank, LargeMotor, OUTPUT_B, OUTPUT_C, MoveDifferential, SpeedRPM, SpeedPercent

from ev3dev2.sensor.lego import ColorSensor, GyroSensor

from ev3dev2.button import Button

from ev3dev2.wheel import EV3Tire

from time import sleep

import math


# Function to move the robot in a rectangular path
def rectangular_path():
    tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)
    track_width = 140  # Distance between the front tyres in mm
    left_motor = OUTPUT_B  # Left motor port
    right_motor = OUTPUT_C  # Right motor port
    wheel_class = EV3Tire  # Tyre type

    # To make a move differential object
    mdiff = MoveDifferential(left_motor, right_motor, wheel_class, track_width)

    # To move the robot in a straight line for 400mm at 40% power
    mdiff.on_for_distance(SpeedRPM(40), 400)

    # To turn the robot 90 degrees at 5% power
    tank_pair.turn_degrees(speed=SpeedPercent(5),target_angle=90)

    # To move the robot in a straight line for 200mm at 40% power
    mdiff.on_for_distance(SpeedRPM(40), 200)

    # To turn the robot 90 degrees at 5% power
    tank_pair.turn_degrees(speed=SpeedPercent(5),target_angle=90)

    # To move the robot in a straight line for 400mm at 40% power
    mdiff.on_for_distance(SpeedRPM(40), 400)

    # To turn the robot 90 degrees at 5% power
    tank_pair.turn_degrees(speed=SpeedPercent(5),target_angle=90)

    # To move the robot in a straight line for 200mm at 40% power
    mdiff.on_for_distance(SpeedRPM(40), 200)

    # To turn the robot 90 degrees at 5% power
    tank_pair.turn_degrees(speed=SpeedPercent(5),target_angle=90)

    '''# To move the robot in a square path
    for _ in range(4):
        # To move the robot in a straight line for 400mm at 40% power
        mdiff.on_for_distance(SpeedRPM(40), 400)

        # To turn the robot 90 degrees at 5% power
        tank_pair.turn_degrees(speed=SpeedPercent(5),target_angle=90)'''

   
# Function to move the robot in a rectangular path, method #2
def rectangular_path_2():
    # This is 1/2 the distance between the fron tyres(midpoint)
    # Could also be the full distance between the tyres, try both and keep adjusting
    # Documentation is unclear
    wheel_distance = 50
    mdiff = MoveDifferential(OUTPUT_B, OUTPUT_C, EV3Tire, wheel_distance)

    for _ in range(4):
        # Drive forward 500 mm
        mdiff.on_for_distance(SpeedRPM(40), 500)

        # Rotate 90 degrees clockwise
        mdiff.turn_right(SpeedRPM(40), 90)


# Function to move the robot in a rectangular path, method #3
def rectangular_path_3():
    track_width = 100  # Distance between the tyres in mm
    turn_radius = (2 * math.pi * track_width) / 4

    revolutions = track_width / turn_radius  # To calc the no. of revs required to cover 500mm
    travel_dist = revolutions * 360  # Degrees
    tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)

    '''for _ in range(2):
        # To move forawd
        tank_pair.on_for_rotations(-50, 50, travel_dist)'''


# Function to move the robot in a lemniscate path
def lemniscate_path():
    tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)
    for _ in range(2):
        # To move forawd
        tank_pair.on_for_seconds(50, 50, 5)
        # To turn left
        #tank_pair.on_for_seconds(-50,50, 1)
        # To move forawd
        tank_pair.on_for_seconds(-50, -50, 5)
         # To turn right
        #tank_pair.on_for_seconds(-50,50, 1)


# Function to move the robot in a circular path
def circular_path(r, s, t):
    # 'r' is the radius in cm
    # 's' is the motor speed in %
    # 't' is the time in seconds

    tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)
    pi = math.pi
    c = 2 * pi * r  # Circumference formula
    
    # To Calculate no. of rotations required to complete the circle
    num_rotations = c / (pi * tank_pair.left_motor.count_per_rot)

    tank_pair.on(s, s)
    tank_pair.wait_for_rotations(num_rotations, num_rotations)
    tank_pair.off()


def main():
    tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)
    #single_motor = LargeMotor(OUTPUT_B)

    tank_pair.gyro = GyroSensor()
    tank_pair.gyro.calibrate()
    color_sensor = ColorSensor()

    for _ in range(3):  # Draw each shape 3 times
        rectangular_path()
        #rectangular_path_2()
        #rectangular_path_3()

    for _ in range(3):  # Draw each shape 3 times
        lemniscate_path()

    for _ in range(3):  # Draw each shape 3 times
        circular_path(10, 70, 5)


main()
