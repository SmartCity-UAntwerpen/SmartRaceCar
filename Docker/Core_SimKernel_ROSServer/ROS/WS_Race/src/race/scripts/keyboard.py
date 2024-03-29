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
import curses
from std_msgs.msg import String
from race.msg import drive_param

def keyboardRace():

  # Initializing the keyboard reader
    stdscr = curses.initscr()
    curses.cbreak()
    stdscr.keypad(1)

  # Initializing the ros publisher
    pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
    rospy.init_node('keyboard', anonymous=True)
#    rate = rospy.Rate(10) # 10hz

    stdscr.refresh()
    key = ''

  # Initializing steer & throttle variables
    steer = 0.0
    throttle = 0.0

    while not rospy.is_shutdown() and key != ord('q'):
        key = stdscr.getch()
        stdscr.refresh()

        if key == curses.KEY_UP:
	    if throttle <= 100.0:
	    	throttle = throttle + 0.1
	elif key == curses.KEY_DOWN:
	    if throttle >= -100.0:
	    	throttle = throttle - 0.1
	elif key == curses.KEY_LEFT: 
	    if steer > -100.0:
		steer = steer - 5
	elif key == curses.KEY_RIGHT:
	    if steer < 100.0:
		steer = steer + 5
	elif key == curses.KEY_DC or key == curses.KEY_BACKSPACE:	
	    #Delete key
	    throttle = 0.0
	    steer = 0.0

        msg = drive_param()
	
      # Set message parameters
	msg.steer = steer
	msg.throttle = throttle 
        rospy.loginfo(msg)
        pub.publish(msg)
 #       rate.sleep()

  # If loop is stopped
    curses.endwin()

if __name__ == '__main__':
    try:
        keyboardRace()
    except rospy.ROSInterruptException:
        pass
