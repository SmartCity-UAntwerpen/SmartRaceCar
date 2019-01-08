#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Float32
from race.msg import drive_param

# pub = rospy.Publisher('speed', Float32, queue_size=10)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
speed = 0.0
steer = 0.0
speedReached = False


def callback(data):
    global speed, speedReached, steer, pub
    measuredRPM = data.data
    print("WantedRPM: {}".format(wantedRPM))
    print("MeasuredRPM: {}".format(measuredRPM))
    print("Speed: {}".format(speed))

    if wantedRPM - threshold_down > measuredRPM:
        speed = 8.9
        rospy.loginfo("Speed up!")
    elif wantedRPM + threshold_up < measuredRPM:
        speed = 0.0
        rospy.loginfo("Speed neutral!")

    msg = drive_param()
    msg.steer = steer
    msg.throttle = speed
    pub.publish(msg)

# def callback(data):
#     global speed, speedReached
#     measuredRPM = data.data
#     print("WantedRPM: {}".format(wantedRPM))
#     print("MeasuredRPM: {}".format(measuredRPM))
#     print("Speed: {}".format(speed))
#     # If the wanted RPM is lower than the Measured RPM, we adjust the speed
#     # For safety we also check if the speed has already been reached
#     if wantedRPM > measuredRPM and speedReached is not True:
#         # Speed threshold: 15 keyboard input
#         if speed < 15:
#             speed = speed + 0.1
#         pub.publish(speed)
#     # If the speed has been reached once, but we slow down, the speed can be set again
#     elif wantedRPM < (measuredRPM - 2) and speedReached is True:
#         speedReached = False
#     else:
#         speedReached = True

if __name__ == '__main__':
    rospy.init_node('SpeedControl', anonymous=True)
    if len(sys.argv) < 4:
        print("usage: speedControlHysteresis.py velocity threshold_up threshold_down")
    else:
        vel = float(sys.argv[1])
        threshold_up = float(sys.argv[2])
        threshold_down = float(sys.argv[3])
        # vel = input("What speed do you want to reach (km/h): ")
        # vel = rospy.get_param('~velocity', 0.0)
        # threshold = rospy.get_param('~threshold', 0.0)

        wantedRPM = (vel / 3.6) * 0.1 / 0.297 * 37 / 13
        rospy.Subscriber('RPMCounter', Float32, callback)
        rospy.spin()
