#!/usr/bin/env python

import rospy, math
from race.msg import drive_param
from std_msgs.msg import String
from geometry_msgs.msg import Twist


def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0 or v == 0:
    return 0

  radius = v / omega
  return math.atan(wheelbase / radius)


def cmd_callback(data):
  global wheelbase
  global ackermann_cmd_topic
  global frame_id
  global pub
  global speed
  
  v = data.linear.x
  steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)
  
  throttle = 0.0
  throttle_temp = v / 0.45 * (speed - 1) + 1 # If speed = 4 :: 0->4 becomes 1->4

  if v <= 0.1:
    throttle = 0.0
  else:
    throttle = throttle_temp
 
  steering = steering * -100.0

  if steering > 100.0:
    steering = 100.0
  elif steering < -100.0:
    steering = -100.0

  msg = drive_param()
  msg.steer = steering
  msg.throttle = throttle
  rospy.loginfo(msg)
  pub.publish(msg)
  

if __name__ == '__main__': 
  try:
    
    rospy.init_node('cmd_vel_to_drive_param')
        
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
    driveparam_cmd_topic = rospy.get_param('~driveparam_cmd_topic', '/drive_parameters')
    wheelbase = rospy.get_param('~wheelbase', 0.324)
    frame_id = rospy.get_param('~frame_id', 'odom')
    speed = rospy.get_param('~speed', 2.0)
    
    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
    pub = rospy.Publisher(driveparam_cmd_topic,drive_param,queue_size=10)
    
    rospy.loginfo("Node 'cmd_vel_to_drive_param' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", twist_cmd_topic, driveparam_cmd_topic, frame_id, wheelbase)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
