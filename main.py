#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog

import time
import threading

import trackedbase
import mathutil

ev3 = EV3Brick()

while len(ev3.buttons.pressed()) == 0:
    pass

gyro = GyroSensor(Port.S1)
gyro.reset_angle(90)
head = Motor(Port.A, gears=[8, 40])
drive_base = trackedbase.TrackedBase(Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE), Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE), gyro)
sonic = UltrasonicSensor(Port.S2)
left_bumper = TouchSensor(Port.S4)
right_bumper = TouchSensor(Port.S3)

# Propotional control parameters
desired_distance = 20
Kp = 20.0
base_speed = 480
base_speed_deviation = int(130.0/480.0 * base_speed)

def bumper_routine():
    drive_base.drive(0,0)
    if left_bumper.pressed():
        drive_base.turn_angle(-45, base_speed)
        drive_base.drive_distance(5, base_speed)
    elif right_bumper.pressed():
        drive_base.drive_distance(-5, base_speed)
        drive_base.turn_angle(-90, base_speed)


def follow_mline():
    current_x, current_y = drive_base.get_position()
    follow_angle = mathutil.get_angle_to_point(current_x, current_y, 250, 250)
    head.run_target(360, 0, wait=False)
    drive_base.turn_to_angle(follow_angle, base_speed)
    trend_detector = mathutil.TrendDetector()
    was_near_goal = False
    while sonic.distance()/10 > 12 and not right_bumper.pressed() and not left_bumper.pressed():
        current_x, current_y = drive_base.get_position()
        distance_from_goal = mathutil.magni2d(current_x - 250, current_y - 250)
        distance_from_goal_increasing = trend_detector.add_number(distance_from_goal)
        if distance_from_goal < 20 and distance_from_goal_increasing:
            drive_base.drive(0,0)
            return True
        head.track_target(0)
        delta_angle = mathutil.looping_minimal_angle_difference(follow_angle, drive_base.get_heading())
        if distance_from_goal > 20:
            drive_base.drive(base_speed - delta_angle * 10, base_speed + delta_angle * 10)
        else:
            was_near_goal = True
            drive_base.drive(base_speed/4 - delta_angle*2.5, base_speed/4 + delta_angle*2.5)
        time.sleep(0.01)
    return was_near_goal

min_distance_from_goal = 999
def follow_obstacle():
    global min_distance_from_goal
    head.run_target(360, 90, wait=False)
    drive_base.turn_angle(-90, base_speed)
    crossed_mline = False
    hitpoint_x, hitpoint_y = drive_base.get_position()
    old_x, old_y = drive_base.get_position()
    current_x, current_y = drive_base.get_position()
    distance_from_goal = 1000
    while not crossed_mline or distance_from_goal > min_distance_from_goal - 10:
        head.track_target(90)

        # MM to CM
        measured_distance = sonic.distance()/10
        # Error from where we want to be
        error = measured_distance - desired_distance

        # Proportional control for where we want to be, adjusts turning
        left_speed = max(min(base_speed - Kp * error, base_speed + base_speed_deviation), base_speed - base_speed_deviation)
        right_speed = max(min(base_speed + Kp * error, base_speed + base_speed_deviation), base_speed - base_speed_deviation)
        drive_base.drive(left_speed, right_speed)
        if left_bumper.pressed() or right_bumper.pressed():
            bumper_routine()
        time.sleep(0.01)

        current_x, current_y = drive_base.get_position()
        if mathutil.magni2d(current_x - hitpoint_x, current_y - hitpoint_y) > 10:
            crossed_mline = mathutil.crossing_mline(old_x, old_y, current_x, current_y)
        old_x, old_y = current_x, current_y
        distance_from_goal = mathutil.magni2d(current_x - 250, current_y - 250)
    if min_distance_from_goal > distance_from_goal:
        min_distance_from_goal = distance_from_goal

drive_base.drive_distance(17.78, base_speed)
drive_base.reset_position(x = 50, y = 0)
at_goal = False

while True:
    at_goal = follow_mline()
    if at_goal:
        break
    follow_obstacle()

drive_base.turn_to_angle(51, base_speed)

ev3.speaker.beep()
