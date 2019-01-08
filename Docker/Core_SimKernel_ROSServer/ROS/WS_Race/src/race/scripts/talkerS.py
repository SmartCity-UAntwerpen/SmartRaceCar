#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from time import sleep
from std_msgs.msg import String
from race.msg import drive_param
from race.msg import drive_values

key = drive_values()
key.pwm_angle = 9831.0
key.pwm_drive = 9831.0
obs = "clear"
brake = "NoBrake"

pub = rospy.Publisher('drive_pwm',drive_values,queue_size=10)

def KeyboardData(data):
    global key
    key = drive_values()
    key.pwm_angle =((data.steer+100)*32.77)+6554  # min 6554 max 13108
    key.pwm_drive =((data.throttle+100)*32.77)+6554
    drive()        

def ObstructionData(data):
    global obs
    obs = data.data
    drive()
   
def BrakeData(data):
    global brake
    brake = data.data
    drive()

def drive():
    global key, brake, obs
    i = 0
    if brake == "brake": #ignore keyboard input
       print("Emergency braking")
       temp = drive_values()
       temp.pwm_angle =11000.0 # tijdens het remmen wordt het wiel gedraait om actief remmen te tonen kan eruit
       temp.pwm_drive =9200.0 # met deze waarde kan gespeeld worden lager getal is niet gelijk aan beter stoppen. laagste tot nu toe getest is 9200
       print(temp)
       pub.publish(temp)
    elif brake == "NoBrake": #listen to keyboard input
        if obs == "clear":
            print("path is clear")
            print(key)
            pub.publish(key)
        elif key.pwm_drive < 9831: # driving backwards
            print("backwards")
            pub.publish(key)
        else: 
            print("neutral") # car completely neutral
            temp = drive_values()
            temp.pwm_angle = 9831.0
            temp.pwm_drive = 9831.0
            pub.publish(temp)

              
        

def listener():
    print("Talker started")
    rospy.init_node('keyboardlistener',anonymous=True)
    rospy.Subscriber('brake',String, BrakeData)
    rospy.Subscriber('drive_parameters',drive_param, KeyboardData)
    rospy.Subscriber('obstruction', String, ObstructionData)
    drive()   
    rospy.spin()

if __name__ == '__main__':
    listener()
