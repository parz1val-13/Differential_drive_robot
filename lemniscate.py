#!/usr/bin/env python3

from ev3dev2.motor import OUTPUT_B, OUTPUT_C, MoveTank
from ev3dev2.sensor.lego import GyroSensor
import time
import math
import sys

# Define your robot's specific parameters
wheel_diameter = 56  # Wheel diameter in millimeters
track_width = 180    # Track width in millimeters

# Initialize the motors
tank_drive = MoveTank(OUTPUT_B, OUTPUT_C)

# Initialize the gyro sensor
gyro = GyroSensor()

# Calibrate the gyro sensor (place the robot in a known starting orientation)
gyro.mode = 'GYRO-CAL'

# Input array: [left_power, right_power, duration]
command = [
    [80, 60, 2],  # Move forward
    [60, 60, 1],  # Move forward
    [-50, 80, 2],  # Move backward
]

# Initialize variables to track the robot's position and orientation
x = 0  # Initial x-coordinate
y = 0  # Initial y-coordinate
theta = 0  # Initial orientation angle (in degrees)

# Initialize ICC variables
icc_x = 0
icc_y = 0

# Encoder resolution (adjust as needed)
encoder_resolution = 360  # Assuming one rotation corresponds to 360 encoder counts

# Iterate through each row in the command array
for row in command:
    left_power, right_power, duration = row

    # Convert percentage to motor speed (fine-tune these values)
    left_speed = int(left_power)
    right_speed = int(right_power)
    duration = int(duration)

    # Reset encoder positions at the start of each movement
    tank_drive.reset()

    # Apply power to the motors and measure encoder counts
    tank_drive.on_for_seconds(left_speed=left_speed, right_speed=right_speed, seconds=duration)

    # Read the gyro sensor's rate of rotation
    rotation_rate = gyro.rate

    # Update the robot's orientation
    theta += rotation_rate * duration

    # Read encoder values
    left_encoder = tank_drive.left_motor.position
    right_encoder = tank_drive.right_motor.position

    # Calculate the displacements in millimeters
    delta_left = (left_encoder / encoder_resolution) * (math.pi * wheel_diameter)
    delta_right = (right_encoder / encoder_resolution) * (math.pi * wheel_diameter)

    # Calculate the forward (X) displacement
    delta_distance = (delta_left + delta_right) / 2.0
    delta_x = delta_distance * math.cos(math.radians(theta))
    delta_y = delta_distance * math.sin(math.radians(theta))

    # Update the robot's position
    x += delta_x
    y += delta_y

# Stop the motors when the sequence is complete
tank_drive.off()

# Print the final location and orientation in centimeters
print("Final Location: X: {:.2f} cm, Y: {:.2f} cm".format(x / 10, y / 10), file=sys.stderr)
print("Final Orientation: Theta: {:.2f} degrees".format(theta), file=sys.stderr)
