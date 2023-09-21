#!/usr/bin/env python3

from ev3dev2.motor import MoveTank, LargeMotor, OUTPUT_B, OUTPUT_C, MoveDifferential, SpeedRPM, SpeedPercent

from ev3dev2.sensor.lego import ColorSensor, GyroSensor

from ev3dev2.button import Button

from ev3dev2.wheel import EV3Tire

#from pybricks.robotics import DriveBase

#from pybricks.parameters import Stop

from time import sleep

import math

import sys

# Function to determine straight line error
def move_straight_error():
    gyro_sensor = GyroSensor()
    gyro_sensor.reset()
    gyro_sensor.mode = 'GYRO-ANG'  # Set the gyro sensor mode to measure angles.
    init_angle = gyro_sensor.angle
    print(init_angle, file=sys.stderr)
    tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)
    tank_pair.on_for_seconds(30,30,3)
    print(gyro_sensor.angle, file=sys.stderr)
    sleep(10)


# Function to determine straight line error
def rotation_error():
    gyro_sensor = GyroSensor()
    gyro_sensor.reset()
    gyro_sensor.mode = 'GYRO-ANG'  # Set the gyro sensor mode to measure angles.
    init_angle = gyro_sensor.angle
    print(init_angle, file=sys.stderr)
    tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)

    wheel_radius = 26
    track_width = 180  # Distance between the tyres in mm
    wheel_circumference = 2 * math.pi * wheel_radius
    turn_distance = ( math.pi * track_width) / 4

    revolutions = turn_distance / wheel_circumference  # To calc the no. of revs required to turn
 
    tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)

    tank_pair.on_for_rotations(25, -25, revolutions)
    print(gyro_sensor.angle, file=sys.stderr)
    sleep(5)


# Function to move the robot in a rectangular path, method 1
def rectangular_path():
    wheel_radius = 26
    track_width = 180  # Distance between the tyres in mm
    wheel_circumference = 2 * math.pi * wheel_radius
    turn_distance = ( math.pi * track_width) / 4

    revolutions = turn_distance / wheel_circumference  # To calc the no. of revs required to turn
 
    tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)

    #tank_pair.on_for_rotations(50, 0, revolutions)

    tank_pair.on_for_seconds(30,30,2)  # To go straight for 2 sec
    sleep(1)  # To stop the robot before turning
    tank_pair.on_for_rotations(25, -25, revolutions) # To turn
    tank_pair.on_for_seconds(30,30,1)  # To go straight for 1 sec
    sleep(1)  # To stop the robot before turning
    tank_pair.on_for_rotations(25, -25, revolutions) # To turn
    tank_pair.on_for_seconds(30,30,2)  # To go straight for 2 sec
    sleep(1)  # To stop the robot before turning
    tank_pair.on_for_rotations(25, -25, revolutions) # To turn
    tank_pair.on_for_seconds(30,30,1)  # To go straight for 1 sec
    sleep(1)  # To stop the robot before turning
    tank_pair.on_for_rotations(25, -25, revolutions) # To turn


# Function to move the robot in a lemniscate path
def lemniscate_path():        
    # wheel parameters (in millimeters)
    wheel_diameter = 56  # Replace with your wheel diameter
    track_width = 180   # Replace with your track width

    # diameter of the circular path (50mm)
    circle_diameter = 200

    # Calculate the radius from the diameter
    circle_radius = circle_diameter / 2.0

    # Calculate the circumference of the wheel
    wheel_circumference = math.pi * wheel_diameter

    # Calculate the circumference of the circle
    circle_circumference = math.pi * circle_diameter

    # Calculate the number of rotations for one lap of the circle
    circle_rotations = circle_circumference / wheel_circumference

    # Calculate the speed for both motors
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


def follow_command():
    # Parameters
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

    # Encoder resolution
    encoder_resolution = 360  # Assuming one rotation corresponds to 360 encoder counts

    # Iterate through each row in the command array
    for row in command:
        left_power, right_power, duration = row

        # Convert str to int
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


# Function to model Braitenberg vehicle, to show cowardice
def Cowardice():
    left_sensor = ColorSensor('in1')
    right_sensor = ColorSensor('in3')
    base_speed = 15
    tank_drive = MoveTank(OUTPUT_B, OUTPUT_C)

    while True:
        left_intensity = left_sensor.ambient_light_intensity
        right_intensity = right_sensor.ambient_light_intensity
        intensity_difference = left_intensity - right_intensity
        speed_difference = intensity_difference * 5
        # Calculate the turn ratio to adjust the robot's direction
        # Calculate the left and right motor speeds
        left_speed = base_speed + speed_difference
        right_speed = base_speed - speed_difference

        # Limit the motor speeds to prevent them from going beyond the allowed range
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))

        # Set the motor speeds
        if abs(intensity_difference) > 15:
            tank_drive.on_for_seconds(-70, -70, .4)
        else:

            tank_drive.on(-left_speed, -right_speed)

        # Sleep for a short time to avoid excessive CPU usage
        sleep(0.1)


# Function to model Braitenberg vehicle, to show aggression
def Aggression():
    left_sensor = ColorSensor('in1')
    right_sensor = ColorSensor('in3')
    base_speed = 10

    tank_drive = MoveTank(OUTPUT_B, OUTPUT_C)

    while True:
        left_intensity = left_sensor.ambient_light_intensity
        right_intensity = right_sensor.ambient_light_intensity
        intensity_difference = left_intensity - right_intensity
        speed_difference = intensity_difference * 5
        # Calculate the left and right motor speeds
        left_speed = base_speed - speed_difference
        right_speed = base_speed + speed_difference

        # Limit the motor speeds to prevent them from going beyond the allowed range
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))

        # Set the motor speeds
        tank_drive.on(left_speed, right_speed)

        # Sleep for a short time to avoid excessive CPU usage
        sleep(0.1)



def main():
    tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)
    #single_motor = LargeMotor(OUTPUT_B)
    #color_sensor = ColorSensor()

    # Answers for question 2
    move_straight_error()
    rotation_error()

    
    # Answer for question 3
    for _ in range(3):  # Draw rectangle shape 3 times
        rectangular_path()

    lemniscate_path()


    # Answer for question 4
    follow_command()


    #Answer for question 5
    Cowardice()
    Aggression()
    
main()


# Function to move the robot in a rectangular path using method #2 - wip
'''def rectangular_path():
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
    tank_pair.on_for_degrees(40, 40, 90)

    # To move the robot in a straight line for 200mm at 40% power
    mdiff.on_for_distance(SpeedRPM(40), 200)

    # To turn the robot 90 degrees at 5% power
    tank_pair.on_for_degrees(40, 40, 90)
    # To move the robot in a straight line for 400mm at 40% power
    mdiff.on_for_distance(SpeedRPM(40), 400)

    # To turn the robot 90 degrees at 5% power
    tank_pair.on_for_degrees(40, 40, 90)
    # To move the robot in a straight line for 200mm at 40% power
    mdiff.on_for_distance(SpeedRPM(40), 200)

    # To turn the robot 90 degrees at 5% power
    tank_pair.on_for_degrees(40, 40, 90)

    # To move the robot in a square path
    for _ in range(4):
        # To move the robot in a straight line for 400mm at 40% power
        mdiff.on_for_distance(SpeedRPM(40), 400)

        # To turn the robot 90 degrees at 5% power
        tank_pair.turn_degrees(speed=SpeedPercent(5),target_angle=90)'''

   
# Function to move the robot in a rectangular path, method #3 - wip
'''def rectangular_path_2():
    # This is 1/2 the distance between the fron tyres(midpoint)
    # Could also be the full distance between the tyres, try both and keep adjusting
    # Documentation is unclear
    wheel_distance = 175
    mdiff = MoveDifferential(OUTPUT_B, OUTPUT_C, EV3Tire, wheel_distance)

    for _ in range(4):
        # Drive forward 500 mm
        mdiff.on_for_distance(SpeedRPM(40), 500)

        # Rotate 90 degrees clockwise
        mdiff.turn_right(SpeedRPM(40), 90)'''


# Function to move the robot in a lemniscate path, method #2
'''def lemniscate_path():
    #tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)

    # Define wheel parameters 
    wheel_diameter = 56  

    # size of the lemniscate
    a = 5   

    # Initialize the motors
    tank_drive = MoveTank(OUTPUT_B, OUTPUT_C)

    # Set the starting angle 't' to 0
    t = 0

    # Set the loop parameters
    max_t = 2 * math.pi  for the desired number of loops
    delta_t = 10  # Time step for simulation

    while t <= max_t:
        # Calculate the x and y coordinates for the current 't'
        x = a * math.cos(t) / math.sqrt(1 + math.sin(t)**2)
        y = a * math.sin(t) * math.cos(t) / math.sqrt(1 + math.sin(t)**2)

        # Calculate the desired distance to move based on the change in x and y
        if t > 0:
            delta_x = x - prev_x
            delta_y = y - prev_y
            distance = math.sqrt(delta_x**2 + delta_y**2)
        else:
            distance = 0

        # Calculate the number of rotations for each wheel to achieve the desired distance
        rotations = distance / (math.pi * wheel_diameter)

        # Calculate the speed for both motors
        speed = 100

        # Set the motors to rotate for the calculated number of rotations
        tank_drive.on_for_rotations(left_speed=speed, right_speed=speed, rotations=rotations)

        # Store the current x and y values for the next iteration
        prev_x = x
        prev_y = y

        # Increment 't' for the next step
        t += delta_t

    # Close the motors
    tank_drive.off()'''


# Function to move the robot in a lemniscate path, #3
'''def lemniscate_path_2():
    tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)
    # Set the number of segments 
    num_segments = 4
    wheel_diameter = 56
    # Set the speed for both motors 
    speed = 50

    # Calculate the distance for each straight segment
    straight_distance = 15 * math.pi / (2 * num_segments)

    # Calculate the number of rotations for each wheel to achieve the straight segment
    rotations = straight_distance / (math.pi * wheel_diameter)

    # Perform the path segments
    for _ in range(num_segments):
        # Move forward in a straight line
        tank_pair.on_for_rotations(left_speed=speed, right_speed=speed, rotations=rotations)

        # Pause briefly to ensure stability 
        tank_pair.off()
        tank_pair.on_for_seconds(left_speed=0, right_speed=0, seconds=0.5)

        # Turn in a circular arc
        tank_pair.on(left_speed=speed, right_speed=0)'''


# Function to move the robot in a circular path
'''def circular_path(r, s, t):
    # 'r' is the radius in cm
    # 's' is the motor speed in %
    # 't' is the time in seconds

    tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)
    pi = math.pi
    c = 2 * pi * r  # Circumference formula
    
    # To Calculate no. of rotations required to complete the circle
    num_rotations = c / (pi * tank_pair.)

    tank_pair.on(s, s)
    tank_pair.wait_for_rotations(num_rotations, num_rotations)
    tank_pair.off()'''


# dead reckoning position controller, method#2
'''def follow_command(command):
    tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)
    tank_pair.gyro = GyroSensor()
    tank_pair.gyro.reset()
    tank_pair.gyro.calibrate()

    track_width = 180  # Distance between the tyres in mm
    est_x = 0.0
    est_y = 0.0
    est_angle = 0.0
    wheel_radius = 28
    max_rpm = (160 + 170) / 2
    max_rps = max_rpm / 60
    circumference = 2 * math.pi * wheel_radius
   
    for row in command:
        power_left = row[0]
        print(power_left)
        power_right = row[1]
        print(power_right)
        duration = row[2]
        print(duration)


        tank_pair.on_for_seconds(power_left, power_right, duration)
        angular_velocity = tank_pair.gyro.rate

        change_in_angle = angular_velocity * duration
        est_angle += change_in_angle
        dist_traveled = change_in_angle * (track_width / 2)

        est_x += dist_traveled * math.cos(math.radians(est_angle))
        est_y += dist_traveled * math.sin(math.radians(est_angle))

        print("Final Estimated Position (X, Y): " + str(est_x) + "mm", + str(est_y) + "mm")
        print("Final Estimated Orientation: " + est_angle + "degrees")

        #gyro_data = tank_pair.gyro.angle

        #left_wheel_dist = power_left * max_rps * circumference
        #right_wheel_dist = power_right * max_rps * circumference
        #avg_dist = (left_wheel_dist + right_wheel_dist) / 2
        #est_x += avg_dist * math.cos(math.radians(gyro_data))
        #est_y += avg_dist * math.sin(math.radians(gyro_data))
        #print(f"Final Estimated Position (X, Y): ({est_x:.2f} mm, {est_y:.2f} mm)")
        '''



