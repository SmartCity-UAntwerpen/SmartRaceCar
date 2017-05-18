from threading import Thread
import os


def start_navstack_thread(currentmap, speed):
    newthread = NavStackThread(currentmap, speed)
    newthread.daemon = False
    newthread.start()


class NavStackThread(Thread):
    def __init__(self, _CURRENTMAP_, _SPEED_):
        Thread.__init__(self)
        self._CURRENTMAP_ = _CURRENTMAP_
        self._SPEED_ = _SPEED_

    def run(self):
        command = "roslaunch f1tenth_2dnav move_base.launch map_name:=" + self._CURRENTMAP_ + \
                  ".yaml speed:=" + str(self._SPEED_) + ""
        os.system(command)


def start_java_thread(startpoint):
    newthread = JavaThread(startpoint)
    newthread.daemon = False
    newthread.start()


class JavaThread(Thread):
    def __init__(self, _STARTPOINT_):
        Thread.__init__(self)
        self._STARTPOINT_ = _STARTPOINT_

    def run(self):
        command = "java -jar ../../../../../SmartRacecar/release/Core/Core.jar " + str(self._STARTPOINT_)
        os.system(command)
