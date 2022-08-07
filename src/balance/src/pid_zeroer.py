#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Bool

def balancing_listener(balancing):
    global motor_speed_pub
    if (balancing == False):
        motor_speed_pub.publish(0.0)

def main():
    rospy.init_node('pid_zeroer')
    rospy.Subscriber('/balancing', Bool, balancing_listener)
    motor_speed_pub = rospy.Publisher('/set_speed_both_motors', Float64, queue_size=10)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()