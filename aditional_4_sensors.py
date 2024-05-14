#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                  InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.B) 
right_motor = Motor(Port.C)

# Initialize the color sensors.
right_sensor = ColorSensor(Port.S4)
left_sensor = ColorSensor(Port.S1)
mid_right_sensor = ColorSensor(Port.S3)
mid_left_sensor = ColorSensor(Port.S2)

robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Set the gains of the PID controller.
PROPORTIONAL_GAIN = 2
INTEGRAL_GAIN = 0.01
DERIVATIVE_GAIN = 30
DRIVE_SPEED = 100

# Initialize variables for integral and previous deviation.
integral = 0
previous_deviation = 0


def sensor_calibration():
    
    S1 = left_sensor.reflection()
    S4 = right_sensor.reflection()
    S2 = mid_right_sensor.reflection()
    S3 = mid_left_sensor.reflection()

    if(S1>S4):
        A = S1-S4
        B = 0
    else:
        A = 0
        B = S4-S1
    if(S2>S3):
        C = S2-S3
        D = 0
    else:
        D = S3-S2
        C = 0



def forward():
    robot.drive(DRIVE_SPEED, 0)


def PID(deviation):
    
    # Update the integral term.
    integral = integral + deviation

    # Calculate the derivative term.
    derivative = deviation - previous_deviation

    # Calculate the turn rate using PID formula.
    turn_rate = (
        PROPORTIONAL_GAIN * deviation
        + INTEGRAL_GAIN * integral
        + DERIVATIVE_GAIN * derivative
    )

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    # Update previous deviation for the next iteration.
    previous_deviation = deviation


sensor_calibration()
while True:

    # Read the color sensor values.
    s1_rf = left_sensor.reflection()
    s4_rf = right_sensor.reflection()
    s2_rf = mid_right_sensor.reflection()
    s3_rf = mid_left_sensor.reflection()

    # Calculate the deviation from the threshold.
    deviation = (((s1_rf - A)-(s4_rf - B))+((s2_rf - C)-(s3_rf - D)))

    if((S1_rf and S2_rf and S3_rf and S4_rf ) > 50 ):
        forward()
    else:
        PID(deviation)


    # You can wait for a short time or do other things in this loop.
    wait(10)