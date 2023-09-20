#!/usr/bin/env python3

from ev3dev2.motor import OUTPUT_B, OUTPUT_C, MoveTank
import math

# Define your wheel parameters (in millimeters)
wheel_diameter = 56  # Replace with your wheel diameter
track_width = 180   # Replace with your track width

# Specify the diameter of the circular path (50mm)
circle_diameter = 200

# Calculate the radius from the diameter
circle_radius = circle_diameter / 2.0

# Calculate the circumference of the wheel
wheel_circumference = math.pi * wheel_diameter

# Calculate the circumference of the circle
circle_circumference = math.pi * circle_diameter

# Calculate the number of rotations for one lap of the circle
circle_rotations = circle_circumference / wheel_circumference

# Calculate the speed for both motors (adjust as needed)
speed_left = 30  # Adjust left motor speed
speed_right = (circle_radius / (circle_radius + (track_width / 2))) * 30  # Adjust right motor speed

# Initialize the motors
tank_drive = MoveTank(OUTPUT_B, OUTPUT_C)

turn_distance = ( math.pi * track_width) / 6

revolutions = turn_distance / wheel_circumference

# Start turning in an arc for one full circle
tank_drive.on_for_rotations(left_speed=speed_left, right_speed=speed_right, rotations=circle_rotations * 2)

tank_drive.on_for_rotations(25, -25, revolutions)

tank_drive.on_for_seconds(30,30,5.1)

tank_drive.on_for_rotations(-25, 25, revolutions)

tank_drive.on_for_rotations(left_speed=speed_right, right_speed=speed_left, rotations=circle_rotations * 1.9)

turn_distance = ( math.pi * track_width) / 8

revolutions = turn_distance / wheel_circumference

tank_drive.on_for_rotations(-25, 25, revolutions)

tank_drive.on_for_seconds(30,30,5)

turn_distance = ( math.pi * track_width) / 6

revolutions = turn_distance / wheel_circumference

tank_drive.on_for_rotations(25, -25, revolutions)


# Close the motors
tank_drive.off()
