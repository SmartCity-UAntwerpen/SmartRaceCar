from random import randint
import cherrypy
import handlers.calc_cost_sim as calccostsim
import json
from handlers.location import Location
import signal
import sys


class SimRest():
    def __init__(self):

        cherrypy.server.socket_host = '0.0.0.0'
        cherrypy.config.update({'server.socket_port': 8084})
        cherrypy.tree.mount(CalculateCost(), '/calcWeight', {'/': {'tools.gzip.on': True}})
        cherrypy.engine.start()

    def stop(self):
        cherrypy.engine.exit()


class CalculateCost:

    def __init__(self):
        pass

    @cherrypy.expose
    @cherrypy.tools.json_out()
    @cherrypy.tools.json_in()
    def index(self):

        print "[CALCCOST][1] Calccost executing..."

        jsonreq = cherrypy.request.json

        print "[CALCCOST][2] Incoming JSON-Request:"
        print jsonreq
        print "------"

        current_location = Location(jsonreq[0]['x'], jsonreq[0]['y'], 0.0, 0.0, 0.0,
                                    jsonreq[0]['z'], jsonreq[0]['w'])

        print "[CALCCOST][3] Current location:"
        print str(current_location)
        print "------"

        start_location = Location(jsonreq[1]['x'], jsonreq[1]['y'], 0.0, 0.0, 0.0,
                                  jsonreq[1]['z'], jsonreq[1]['w'])

        print "[CALCCOST][4] Start location:"
        print str(start_location)
        print "------"

        goal_location = Location(jsonreq[2]['x'], jsonreq[2]['y'], 0.0, 0.0, 0.0,
                                 jsonreq[2]['z'], jsonreq[2]['w'])

        print "[CALCCOST][5] Start location:"
        print str(goal_location)
        print "------"

        current_posestamped = calccostsim.pose_2_posestamped(calccostsim.location_2_pose(current_location))
        start_posestamped = calccostsim.pose_2_posestamped(calccostsim.location_2_pose(start_location))
        goal_posestamped = calccostsim.pose_2_posestamped(calccostsim.location_2_pose(goal_location))

        print "[CALCCOST][6] Posestamped calculated"
        print "------"

        time_curstart = calccostsim.delegate_cost(current_posestamped, start_posestamped, 0.3, 4.0)
        time_startgoal = calccostsim.delegate_cost(start_posestamped, goal_posestamped, 0.3, 4.0)

        print "[CALCCOST][7] Time current start: " + str(time_curstart)
        print "[CALCCOST][8] Time start goal: " + str(time_startgoal)

        jsonmessage = {'cost': {'status': False, 'weightToStart': time_curstart,
                                'weight': time_startgoal, 'idVehicle': 12321}}

        print "[CALCCOST][9] Jsonmessage:"
        print jsonmessage
        print "------"

        return jsonmessage


def signal_handler(signal, frame):
    rossim.stop()
    sys.exit(0)

if __name__ == "__main__":
    calccostsim.init_ros()
    rossim = SimRest()
    signal.signal(signal.SIGINT, signal_handler)
    print "Press Ctrl+C to exit"
    signal.pause()
