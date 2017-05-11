from location import Location

import rospy
from race.msg import drive_param
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionFeedback


'''
Global variables are made in 'init_ros()':
    logger
    pub_drive_parameters
    pub_initial_pose
    pub_movebase_goal
'''

cb_movebase_feedback_secs = 0


def publish_drive_param(json_string):
    msg = drive_param()
    msg.steer = json_string['drive']['steer']
    msg.throttle = json_string['drive']['throttle']
    pub_drive_parameters.publish(msg)
    logger.log_debug("[ROSMODULE] drive_param message sent")
    return


def stop():
    msg = drive_param()
    msg.steer = 0
    msg.throttle = 0
    pub_drive_parameters.publish(msg)
    logger.log_info("[ROSMODULE] Stop signal sent")
    return


def publish_initialpose(location):
    i = 0
    while i < 3:
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = location.posx
        pose.pose.pose.position.y = location.posy
        pose.pose.pose.position.z = location.posz
        pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.06853891945200942]
        pose.pose.pose.orientation.x = location.orx
        pose.pose.pose.orientation.y = location.ory
        pose.pose.pose.orientation.z = location.orz
        pose.pose.pose.orientation.w = location.orw

        pub_initial_pose.publish(pose)
        i += 1

    logger.log_debug("[ROSMODULE] Initial pose published three times")
    return


def publish_movebase_goal(posx, posy, posz, orx, ory, orz, orw):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = posx
    pose.pose.position.y = posy
    pose.pose.position.z = posz
    pose.pose.orientation.x = orx
    pose.pose.orientation.y = ory
    pose.pose.orientation.z = orz
    pose.pose.orientation.w = orw

    pub_movebase_goal.publish(pose)
    logger.log_debug("[ROSMODULE] Goal published")
    return


def cb_movebase_status(data):
    status_list = data.status_list
    if len(status_list) != 0:
        logger.log_debug("[STATUS] Status: " + status_list[len(status_list) - 1].status)


def cb_movebase_feedback(data):
    global cb_movebase_feedback_secs
    header = data.header
    if header.stamp.secs - cb_movebase_feedback_secs >= 1:
        cb_movebase_feedback_secs = header.stamp.secs
        pose = data.feedback.base_position.pose
        logger.log_debug("[FEEDBACK] Secs: " + header.stamp.secs)
        logger.log_debug("[FEEDBACK] Position: X: " + pose.position.x +
                         ", Y: " + pose.position.y +
                         ", Z: " + pose.position.z)
        logger.log_debug("[FEEDBACK] Orientation: X: " + pose.orientation.x +
                         ", Y: " + pose.orientation.y +
                         ", Z: " + pose.orientation.z +
                         ", W: " + pose.orientation.w)
        location = Location(pose.position.x, pose.position.y, 0, 0, 0, pose.orientation.z, pose.orientation.w)

        from .. import javalinker
        javalinker.send_location(location)


def init_ros(logger_argument):
    global pub_drive_parameters, pub_initial_pose, pub_movebase_goal, logger

    logger = logger_argument

    pub_drive_parameters = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
    pub_initial_pose = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    pub_movebase_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

    logging_level = logger.logging_level
    if logging_level == 1:
        rospy.init_node('javalinker', log_level=rospy.DEBUG, anonymous=True)
    elif logging_level == 2:
        rospy.init_node('javalinker', log_level=rospy.INFO, anonymous=True)
    elif logging_level == 3:
        rospy.init_node('javalinker', log_level=rospy.WARN, anonymous=True)
    elif logging_level == 4:
        rospy.init_node('javalinker', log_level=rospy.ERROR, anonymous=True)

    rospy.Subscriber('move_base/status', GoalStatusArray, cb_movebase_status)
    rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, cb_movebase_feedback)

    logger.log_info("[ROSMODULE] ROS initialised")


def rospy_spin():
    logger.log_debug("[ROSMODULE] rospy.spin() activated")
    rospy.spin()
