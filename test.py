#!/usr/bin/env python3

# An EV3 Python (library v2) solution to Exercise 3

# of the official Lego Robot Educator lessons that

# are part of the EV3 education software

from ev3dev2.motor import MoveTank, LargeMotor, OUTPUT_B, OUTPUT_C, MoveDifferential, SpeedRPM, SpeedPercent

from ev3dev2.sensor.lego import ColorSensor, GyroSensor

from ev3dev2.button import Button

from ev3dev2.wheel import EV3Tire

#from pybricks.robotics import DriveBase

#from pybricks.parameters import Stop

from time import sleep

import math


def move_straight_error():
    
    gyro_sensor = GyroSensor()
    gyro_sensor.reset()
    gyro_sensor.mode = 'GYRO-ANG'  # Set the gyro sensor mode to measure angles.
    init_angle = gyro_sensor.angle
    print(init_angle)
    tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)
    tank_pair.on_for_seconds(30,30,3)
    print(gyro_sensor.angle)
    sleep(5)

def rotation_error():
    gyro_sensor = GyroSensor()
    gyro_sensor.reset()
    gyro_sensor.mode = 'GYRO-ANG'  # Set the gyro sensor mode to measure angles.
    init_angle = gyro_sensor.angle
    print(init_angle)
    tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)

    wheel_radius = 26
    track_width = 180  # Distance between the tyres in mm
    wheel_circumference = 2 * math.pi * wheel_radius
    turn_distance = ( math.pi * track_width) / 4

    revolutions = turn_distance / wheel_circumference  # To calc the no. of revs required to cover 500mm
 
    tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)

    tank_pair.on_for_rotations(25, -25, revolutions)
    print(gyro_sensor.angle)
    sleep(5)

# Function to move the robot in a rectangular path
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

   
# Function to move the robot in a rectangular path, method #2
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


# Function to move the robot in a rectangular path, method #3
def rectangular_path():
    wheel_radius = 26
    track_width = 180  # Distance between the tyres in mm
    wheel_circumference = 2 * math.pi * wheel_radius
    turn_distance = ( math.pi * track_width) / 4

    revolutions = turn_distance / wheel_circumference  # To calc the no. of revs required to cover 500mm
 
    tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)


    #tank_pair.on_for_rotations(50, 0, revolutions)

    tank_pair.on_for_seconds(30,30,3)
    sleep(1)
    tank_pair.on_for_rotations(25, -25, revolutions)
    tank_pair.on_for_seconds(30,30,1.5)
    sleep(1)
    tank_pair.on_for_rotations(25, -25, revolutions)
    tank_pair.on_for_seconds(30,30,3)
    sleep(1)
    tank_pair.on_for_rotations(25, -25, revolutions)
    tank_pair.on_for_seconds(30,30,1.5)
    sleep(1)
    tank_pair.on_for_rotations(25, -25, revolutions)


# Function to move the robot in a lemniscate path
def lemniscate_path():        
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



# Function to move the robot in a lemniscate path
'''def lemniscate_path():
    #tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)

    # Define wheel parameters 
    wheel_diameter = 56  

    # Specify the desired size of the lemniscate
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


# Function to move the robot in a lemniscate path
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


def follow_command(command):
    tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)
    tank_pair.gyro = GyroSensor()
    tank_pair.gyro.reset()
    tank_pair.gyro.calibrate()

    track_width = 100  # Distance between the tyres in mm
    est_x = 0.0
    est_y = 0.0
    est_angle = 0.0
    r = 28
    max_rpm = (160 + 170) / 2
    max_rps = max_rpm / 60
    circumference = 2 * math.pi * r
   
    for row in command:
        power_left = row[0]
        power_right = row[1]
        duration = row[2]

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
        

def main():
    tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)
    #single_motor = LargeMotor(OUTPUT_B)

    #move_straight()

    #rotation_error()

    #color_sensor = ColorSensor()

    for _ in range(3):  # Draw each shape 3 times
        rectangular_path()
        #rectangular_path_2()
        #rectangular_path_3()

    #for _ in range(3):  # Draw each shape 3 times
    lemniscate_path()

    #for _ in range(3):  # Draw each shape 3 times
        #circular_path(10, 70, 5)

    command = [[80, 60, 2], [60,60,1], [-50, 80, 2]]

    follow_command(command)

main()
