from random import randint
import cherrypy
import json
from handlers.location import Location

# from droneparameters import DroneParameters
# import requests


class SimRest():
    def __init__(self):
        # logg
        # self.id_droneparam=id_droneparam
        # self.mqtt_client= mqtt_client

        cherrypy.server.socket_host = '0.0.0.0'
        cherrypy.tree.mount(CalculateCost(), '/calcWeight', {'/': {'tools.gzip.on': True}})
        cherrypy.engine.start()
        # self.getWaypoints()
        # print self.waypoints[1]['ID']


class CalculateCost:

    def __init__(self):
        pass

    # def _cp_dispatch(self, vpath):
    #     if len(vpath)== 3:
    #         self.start = vpath.pop(0)#start
    #         vpath.pop(0)#to
    #         self.end = vpath.pop(0)#end
    #         return self

    @cherrypy.expose
    @cherrypy.tools.json_out()
    @cherrypy.tools.json_in()
    def index(self):

        print"Starting calccost method"
        print "------"

        jsonreq = cherrypy.request.json

        print "Incoming JSON-Request:"
        print jsonreq
        print "------"

        current_location = Location(jsonreq['cost'][0]['x'], jsonreq['cost'][0]['y'], 0.0, 0.0, 0.0,
                                    jsonreq['cost'][0]['z'], jsonreq['cost'][0]['w'])

        print "Current location:"
        print current_location
        print "------"

        start_location = Location(jsonreq['cost'][1]['x'], jsonreq['cost'][1]['y'], 0.0, 0.0, 0.0,
                                  jsonreq['cost'][1]['z'], jsonreq['cost'][1]['w'])

        print "Start location:"
        print start_location
        print "------"

        goal_location = Location(jsonreq['cost'][2]['x'], jsonreq['cost'][2]['y'], 0.0, 0.0, 0.0,
                                 jsonreq['cost'][2]['z'], jsonreq['cost'][2]['w'])

        print "Start location:"
        print goal_location
        print "------"

        jsonmessage = {'cost': {'status': False, 'weightToStart': 4.0,
                                'weight': 3.0, 'idVehicle': 12321}}

        print "Jsonmessage:"
        print jsonmessage
        print "------"

        return jsonmessage

if __name__ == "__main__":
    rossim = SimRest()