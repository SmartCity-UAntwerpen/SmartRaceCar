from location import Location
from .. import javalinker
import rospy
from race.msg import drive_param
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionFeedback


# class RosClass:
#     def __init__(self):
#         self.pub_drive_parameters = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
#         self.pub_initial_pose = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
#         self.pub_move_base_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
#         rospy.init_node('javalinker', anonymous=True)
#
#         rospy.Subscriber('move_base/status', GoalStatusArray, self.cb_movebase_status)
#         rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, self.cb_movebase_feedback)
#
#     def cb_movebase_status(self):
#         status_list = data.status_list
#         if len(status_list) != 0:
#             print "[STATUS] Status: %d" % status_list[len(status_list) - 1].status
#

pub_drive_parameters = None
pub_initial_pose = None
pub_movebase_goal = None

cb_movebase_feedback_secs = 0


def publish_drive_param(json_string):
    msg = drive_param()
    msg.steer = json_string['drive']['steer']
    msg.throttle = json_string['drive']['throttle']
    rospy.loginfo(msg)
    pub_drive_parameters.publish(msg)
    return


def stop():
    msg = drive_param()
    msg.steer = 0
    msg.throttle = 0
    rospy.loginfo(msg)
    pub_drive_parameters.publish(msg)
    return


def cb_movebase_status(data):
    status_list = data.status_list
    if len(status_list) != 0:
        print "[STATUS] Status: %d" % status_list[len(status_list) - 1].status


def cb_movebase_feedback(data):
    global cb_movebase_feedback_secs
    header = data.header
    if header.stamp.secs - cb_movebase_feedback_secs >= 1:
        cb_movebase_feedback_secs = header.stamp.secs
        pose = data.feedback.base_position.pose
        print "[FEEDBACK] Secs: %d" % (header.stamp.secs)
        print "[FEEDBACK] Position: X: %f, Y: %f, Z: %f" % (pose.position.x, pose.position.y, pose.position.z)
        print "[FEEDBACK] Orientation: X: %f, Y: %f, Z: %f, W: %f" % (pose.orientation.x, pose.orientation.y,
                                                                      pose.orientation.z, pose.orientation.w)
        location = Location(pose.position.x, pose.position.y, 0, 0, 0, pose.orientation.z, pose.orientation.w)
        javalinker.send_location(location)


def init_ros():
    global pub_drive_parameters, pub_initial_pose, pub_movebase_goal
    pub_drive_parameters = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
    pub_initial_pose = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    pub_movebase_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

    rospy.init_node('javalinker', anonymous=True)

    rospy.Subscriber('move_base/status', GoalStatusArray, cb_movebase_status)
    rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, cb_movebase_feedback)