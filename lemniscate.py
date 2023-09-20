#!/usr/bin/env python3

from ev3dev2.motor import OUTPUT_B, OUTPUT_C, MoveTank
import math

# Define your wheel parameters (in centimeters)
wheel_diameter = 56  # Replace with your wheel diameter
track_width = 180   # Replace with your track width

# Specify the radius of the desired arc (in centimeters)
arc_radius = 50    # Adjust this to your desired radius

# Calculate the circumference of the wheels
wheel_circumference = math.pi * wheel_diameter  # Pi * diameter

semi_circle_circumference = math.pi * (arc_radius/2)
# Calculate the speed for both motors (you may need to adjust this)
speed = 50

rotations = semi_circle_circumference / wheel_circumference
# Calculate the angular velocity (angular speed) for the robot
angular_speed = speed / (arc_radius / 2.0)

# Initialize the motors
tank_drive = MoveTank(OUTPUT_B, OUTPUT_C)

# Set the left motor to move slightly slower than the right motor
tank_drive.on_for_rotations(speed - angular_speed, speed, rotations)

'''degrees_to_turn = -45
rotations_to_turn = (degrees_to_turn / 360) * (wheel_circumference / wheel_diameter)

# Turn 45 degrees (adjust speed as needed)
tank_drive.on_for_rotations(left_speed=speed, right_speed=-speed, rotations=rotations_to_turn)

# Go straight for a certain duration (adjust as needed)
straight_duration = 3.0  # 2 seconds as an example, adjust as needed
tank_drive.on_for_seconds(left_speed=speed, right_speed=speed, seconds=straight_duration)'''

'''degrees_to_turn = 45
rotations_to_turn = (degrees_to_turn / 360) * (wheel_circumference / wheel_diameter)

# Turn 45 degrees (adjust speed as needed)
tank_drive.on_for_rotations(left_speed=speed, right_speed=-speed, rotations=rotations_to_turn)

# Set the left motor to move slightly slower than the right motor
tank_drive.on_for_seconds(speed, (speed - angular_speed), 5.23)

degrees_to_turn = 45
rotations_to_turn = (degrees_to_turn / 360) * (wheel_circumference / wheel_diameter)

# Turn 45 degrees (adjust speed as needed)
tank_drive.on_for_rotations(left_speed=speed, right_speed=-speed, rotations=rotations_to_turn)

# Go straight for a certain duration (adjust as needed)
straight_duration = 3.0  # 2 seconds as an example, adjust as needed
tank_drive.on_for_seconds(left_speed=speed, right_speed=speed, seconds=straight_duration)'''


