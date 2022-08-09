#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Empty
import RPi.GPIO as GPIO
import queue
import time

from dataclasses import dataclass, field
from typing import Any, NewType

Pin = NewType('Pin', int)

MOTOR_MAX_FREQUENCY = 1500

class MotorDriver:
    def __init__(self, speed_pin:Pin, direction_pin:Pin, flipped:bool=False):
        GPIO.setup(speed_pin, GPIO.OUT)
        self.__speed__ = 0
        self.__pwm__ = GPIO.PWM(speed_pin, MOTOR_MAX_FREQUENCY)
        GPIO.setup(direction_pin, GPIO.OUT)
        self.__direction_pin__ = direction_pin
        self.__flipped__ = flipped

    def set_speed(self, speed_msg):
        speed = min(1, max(-1, speed_msg.data))
        print("setting speed to: ", speed)
        self.__set_magnitude__(abs(speed))
        self.__set_direction__(speed > 0)
        self.__speed__ = speed

    def __set_magnitude__(self, speed_magnitude):
        speed_was_zero = (self.__speed__ == 0)

        if (speed_magnitude == 0):
            self.__pwm__.stop()
        else:
            self.__pwm__.ChangeFrequency(speed_magnitude * MOTOR_MAX_FREQUENCY)

        if (speed_was_zero and speed_magnitude != 0):
            self.__pwm__.start(50)

    def __set_direction__(self, forward:bool):
        if (forward ^ (self.__flipped__)): # ^ is xor
            GPIO.output(self.__direction_pin__, GPIO.HIGH)
        else:
            GPIO.output(self.__direction_pin__, GPIO.LOW)

    def __delete__(self):
        self.__pwm__.stop()

def main():
    rospy.init_node('gpio')
    GPIO.setmode(GPIO.BCM)

    right_motor = MotorDriver(Pin(4), Pin(14))
    left_motor = MotorDriver(Pin(17), Pin(18), flipped = True)

    rospy.Subscriber('set_speed_right_motor', Float64, right_motor.set_speed)
    rospy.Subscriber('set_speed_left_motor', Float64, left_motor.set_speed)

    def set_speed_both_motors(speed_msg):
        right_motor.set_speed(speed_msg)
        left_motor.set_speed(speed_msg)
    rospy.Subscriber('set_speed_both_motors', Float64, set_speed_both_motors)

    def emergency_stop(_):
        reason = "gpio received emergency stop message. Shutting down."
        rospy.logerr(reason)
        rospy.signal_shutdown(reason)

    rospy.Subscriber('emergency_stop', Empty, emergency_stop)
    rospy.loginfo("Motor services up.")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    del right_motor
    del left_motor
    GPIO.cleanup()

if __name__ == '__main__':
    main()