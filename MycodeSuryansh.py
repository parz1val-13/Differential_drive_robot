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
