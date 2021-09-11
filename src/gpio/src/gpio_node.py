#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import queue
import time

from dataclasses import dataclass, field
from typing import Any, NewType

Pin = NewType('Pin', int)
# R_MOTOR_GPIO = Pin(4)

MOTOR_MAX_FREQUENCY = 1500

class MotorDriver:
    def __init__(self, speed_pin:Pin, direction_pin:Pin):
        GPIO.setup(speed_pin, GPIO.OUT)
        self.__speed__ = 0
        self.__pwm__ = GPIO.PWM(speed_pin, MOTOR_MAX_FREQUENCY)
        GPIO.setup(direction_pin, GPIO.OUT)
        self.__direction_pin__ = direction_pin

    def set_speed(self, speed_msg):
        speed = speed_msg.data
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
        if (forward):
            gpio.output(self.__direction_pin__, gpio.HIGH)
        else:
            gpio.output(self.__direction_pin__, gpio.LOW)

    def __delete__(self):
        self.__pwm__.stop()

def main():
    rospy.init_node('gpio')
    GPIO.setmode(GPIO.BCM)

    right_motor = MotorDriver(Pin(4), Pin(14))

    rospy.Subscriber('set_speed_right_motor', Float32, right_motor.set_speed)
    rospy.loginfo("Motor service up.")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    del right_motor
    GPIO.cleanup()

if __name__ == '__main__':
    main()