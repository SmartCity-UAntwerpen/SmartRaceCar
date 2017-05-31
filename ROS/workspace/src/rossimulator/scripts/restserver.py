from random import randint
import cherrypy
import handlers.calc_cost_sim as calccostsim
import json
from handlers.location import Location


class SimRest():
    def __init__(self):
        # logg
        # self.id_droneparam=id_droneparam
        # self.mqtt_client= mqtt_client

        cherrypy.server.socket_host = '0.0.0.0'
        cherrypy.tree.mount(CalculateCost(self.logger), '/calcWeight', {'/': {'tools.gzip.on': True}})
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

        print "[CALCCOST] Calccost executing..."

        jsonreq = cherrypy.request.json

        print "[CALCCOST] Incoming JSON-Request:"
        print jsonreq
        print "------"

        current_location = Location(jsonreq['cost'][0]['x'], jsonreq['cost'][0]['y'], 0.0, 0.0, 0.0,
                                    jsonreq['cost'][0]['z'], jsonreq['cost'][0]['w'])

        print "[CALCCOST] Current location:"
        print current_location
        print "------"

        start_location = Location(jsonreq['cost'][1]['x'], jsonreq['cost'][1]['y'], 0.0, 0.0, 0.0,
                                  jsonreq['cost'][1]['z'], jsonreq['cost'][1]['w'])

        print "[CALCCOST] Start location:"
        print start_location
        print "------"

        goal_location = Location(jsonreq['cost'][2]['x'], jsonreq['cost'][2]['y'], 0.0, 0.0, 0.0,
                                 jsonreq['cost'][2]['z'], jsonreq['cost'][2]['w'])

        print "[CALCCOST] Start location:"
        print goal_location
        print "------"

        jsonmessage = {'cost': {'status': False, 'weightToStart': 4.0,
                                'weight': 3.0, 'idVehicle': 12321}}

        print "[CALCCOST] Jsonmessage:"
        print jsonmessage
        print "------"

        current_posestamped = calccostsim.pose_2_posestamped(calccostsim.location_2_pose(current_location))
        start_posestamped = calccostsim.pose_2_posestamped(calccostsim.location_2_pose(start_location))
        goal_posestamped = calccostsim.pose_2_posestamped(calccostsim.location_2_pose(goal_location))

        print "[CALCCOST] Posestamped calculated"
        print "------"

        # time = calccostsim.delegate_cost(current_location, start_location, 0.3, 4.0)

        # print "[CALCCOST] Time: " + str(time)

        return jsonmessage

if __name__ == "__main__":
    rossim = SimRest()
