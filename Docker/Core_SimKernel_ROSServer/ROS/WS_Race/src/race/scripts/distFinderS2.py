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
import math
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu

inRange = False
inBrake = False
Brake = "NoBrake"
dist = "clear"
distances = ()
rpm = 0
imu = 0
rpmC = True # controle of de rpm tijdens het remmen 0 is geweest

pub = rospy.Publisher('obstruction', String, queue_size=10)
pub1 = rospy.Publisher('brake', String, queue_size=10)

def ImuData(data):
    global imu
    imu = data.linear_acceleration.y
    braking()

def LidarData(data): #lidar data
    global inRange,distances
    theta1 = 126.46;
    theta2 = 143.54;
    distances = getRange(data.ranges, theta1, theta2)
    obstruction()
    braking()

def RpmData(data): 
    global rpm,rpmC,Brake
    rpm = data.data
    if Brake == "brake" and rpm == 0:
        print("rpmC is False")
        rpmC = False
        braking()

    if rpmC == False and Brake == "NoBrake":
        print("rpmC is True")
        rpmC = True

    obstruction()
    braking()

def getRange(data, theta1, theta2):
    global distances
    rangeA = int(math.floor(theta1/0.25) - 1)
    rangeB = int(math.ceil(theta2/0.25) - 1)
    distances = data[rangeA:rangeB]
    return distances
 
def obstruction():
    global dist, distances,inRange, closer
    i = 0
    if inRange == False:
        tempInRange = False
        while i < (len(distances)-1):
            if distances[i] <= 1.2: # object at 1.2m or closer
                tempInRange = True
            i += 1
        if tempInRange == True:
            inRange = True
            dist = "obs"
            pub.publish(dist)
            print(dist)
    elif inRange == True:
        tempInRange = False
        while i < (len(distances)-1):
            if distances[i] <= 1.2: # object at 1.2m or closer
                tempInRange = True
            i += 1
        if tempInRange == False:
            inRange = False
            dist = "clear"
            pub.publish(dist)
            print(dist)
def braking():
    global inBrake,Brake,closer,imu,rpmC
    i = 0
    if inBrake == False:
        tempInRange = False
        if dist == "clear":#Imu groter dan 1 is achteruit
            tempInRange = True
        if tempInRange == True:
            inBrake= True
            Brake = "NoBrake"
            pub1.publish(Brake)
            print(Brake)
    elif inBrake == True:
        tempInRange = False
        if dist == "obs": # Met de imu waarde kan gespeeld worden maar is niet altijd consequent
            tempInRange = True
        if tempInRange == True:
            inBrake = False
            Brake = "brake"
            pub1.publish(Brake)
            print(Brake)

if __name__ == '__main__':
    print("DistFinder node started!")
    rospy.init_node('distfinder', anonymous = True)
    pub.publish(dist)
    rospy.loginfo(dist)
    rospy.Subscriber("scan", LaserScan, LidarData)
    rospy.Subscriber("RPMCounter",Int32,RpmData)
    rospy.Subscriber("imu",Imu,ImuData)
    rospy.spin()
