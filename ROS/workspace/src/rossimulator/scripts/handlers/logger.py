import logging

try:
    import rospy
    debug_without_ros = False
except ImportError:
    debug_without_ros = True

'''
Logging level:
    1: DEBUG
    2: INFO
    3: WARNING
    4: CRITICAL
'''
logging_level = 1


class Logger:

    def __init__(self):
        self.logging_level = logging_level

        print "logger init"

        if debug_without_ros:
            if self.logging_level == 1:
                logging.basicConfig(level=logging.DEBUG, format='%(asctime)s [%(levelname)s] %(message)s')
            elif self.logging_level == 2:
                logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
            elif self.logging_level == 3:
                logging.basicConfig(level=logging.WARNING, format='%(asctime)s [%(levelname)s] %(message)s')
            elif self.logging_level == 4:
                logging.basicConfig(level=logging.CRITICAL, format='%(asctime)s [%(levelname)s] %(message)s')

    def log_debug(self, text):
        if debug_without_ros:
            logging.debug(text)
        else:
            rospy.logdebug(text)

    def log_info(self, text):
        if debug_without_ros:
            logging.info(text)
        else:
            rospy.loginfo(text)

    def log_warning(self, text):
        if debug_without_ros:
            logging.warning(text)
        else:
            rospy.logwarn(text)

    def log_critical(self, text):
        if debug_without_ros:
            logging.critical(text)
        else:
            rospy.logerr(text)
