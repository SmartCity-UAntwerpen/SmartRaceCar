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
    # @cherrypy.tools.json_in()
    def index(self):

        print "[CALCCOST] Calccost executing..."

        # jsonreq = cherrypy.request.json

        print "[CALCCOST] Incoming JSON-Request:"
        # print jsonreq
        print "------"

        current_location = Location(0.5, 0.0, 0.0, 0.0, 0.0, -1.0, 0.02)
        start_location = Location(-13.4, -0.53, 0.0, 0.0, 0.0, 0.71, 0.71)
        goal_location = Location(-27.14, -1.11, 0.0, 0.0, 0.0, -0.3, 0.95)

        # current_location = Location(jsonreq['cost'][0]['x'], jsonreq['cost'][0]['y'], 0.0, 0.0, 0.0,
        #                             jsonreq['cost'][0]['z'], jsonreq['cost'][0]['w'])

        print "[CALCCOST] Current location:"
        print current_location
        print "------"

        # start_location = Location(jsonreq['cost'][1]['x'], jsonreq['cost'][1]['y'], 0.0, 0.0, 0.0,
        #                           jsonreq['cost'][1]['z'], jsonreq['cost'][1]['w'])

        print "[CALCCOST] Start location:"
        print start_location
        print "------"

        # goal_location = Location(jsonreq['cost'][2]['x'], jsonreq['cost'][2]['y'], 0.0, 0.0, 0.0,
        #                          jsonreq['cost'][2]['z'], jsonreq['cost'][2]['w'])

        print "[CALCCOST] Start location:"
        print goal_location
        print "------"

        current_posestamped = calccostsim.pose_2_posestamped(calccostsim.location_2_pose(current_location))
        start_posestamped = calccostsim.pose_2_posestamped(calccostsim.location_2_pose(start_location))
        goal_posestamped = calccostsim.pose_2_posestamped(calccostsim.location_2_pose(goal_location))

        print "[CALCCOST] Posestamped calculated"
        print "------"

        # time_curstart = calccostsim.delegate_cost(current_posestamped, start_posestamped, 0.3, 4.0)
        # time_startgoal = calccostsim.delegate_cost(start_posestamped, goal_posestamped, 0.3, 4.0)

        # print "[CALCCOST] Time curstart: " + str(time_curstart)
        # print "[CALCCOST] Time startgoal: " + str(time_startgoal)

        jsonmessage = {'cost': {'status': False, 'weightToStart': 321.0,
                                'weight': 123.0, 'idVehicle': 12321}}

        print "[CALCCOST] Jsonmessage:"
        print jsonmessage
        print "------"

        return jsonmessage

if __name__ == "__main__":
    calccostsim.init_ros()
    rossim = SimRest()

'''
<x>0.5</x>
<y>0</y>
<z>-1</z>
<w>0.02</w>

<x>-13.4</x>
<y>-0.53</y>
<z>0.71</z>
<w>0.71</w>

<x>-27.14</x>
<y>-1.11</y>
<z>-0.3</z>
<w>0.95</w>

<x>-28.25</x>
<y>-9.19</y>
<z>-0.71</z>
<w>0.71</w>
'''