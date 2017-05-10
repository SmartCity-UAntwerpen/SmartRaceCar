import logging

try:
    import rospy
    debug_without_ros = False
except ImportError:
    debug_without_ros = True


class Logger:

    def __init__(self):
        global debug_without_ros
        print "logger"

        if debug_without_ros:
            logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')

    def log_debug(self, text):
        global debug_without_ros

        if debug_without_ros:
            logging.debug(text)
        else:
            rospy.logdebug(text)

    def log_info(self, text):
        global debug_without_ros

        if debug_without_ros:
            logging.info(text)
        else:
            rospy.loginfo(text)

    def log_warning(self, text):
        global debug_without_ros

        if debug_without_ros:
            logging.warning(text)
        else:
            rospy.logwarn(text)

    def log_critical(self, text):
        global debug_without_ros

        if debug_without_ros:
            logging.critical(text)
        else:
            rospy.logerr(text)
