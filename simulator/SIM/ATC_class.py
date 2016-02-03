'''
Package: atc
module: atc_class
module dependence: Collision_class

description:
Eleborate description: (link to formulas)

Input:
Output:
'''

#import python modules
from math import *
import random as rnd
import networkx as nx
from weightedCustom import dijkstra_path

#import OSSAR modules
# from dijkstra_structure import shortestpath
from command_class import *

#import data
from data_import import wp_database
from data_import import wpl_database
from data_import import min_dec #minimum deceleration
#from collision_class import collision

##create instances
#coll_ins = collision()

class ATC:
    def __init__(self, ATC_ID, ATClink, locplanes, ATCtype, x_handoff, y_handoff):
        self.id = ATC_ID                # ATC identification number
        self.link = ATClink             # list of possible handover ATC
        self.locp = locplanes           # Array of planes under command
        self.type = ATCtype             # Type of ATC (gate(1), waypoint(2), end(4))
        self.x_handoff = x_handoff      # x-coordinate at which a aircraft should be handed over
        self.y_handoff = y_handoff      # y-coordinate at which a aircraft should be handed over
        self.throughput = False         # thoughput of ATC
        self.par = []

    # def update_aircraft_distance(self):
    #     for thisPlane in self.locp:
    #         thisPlane.


    # check if commands for the plane are necessary and update planned operation
    def update(self,ATC_list,runway_list,v_max,graph,runway_occupance_time,dt,t):
        graph = self.command_check(ATC_list,runway_list,v_max,graph,runway_occupance_time,dt,t)
        return graph
    
    #check if a plane needs a command
    def command_check(self,ATC_list,runway_list,v_max,graph,runway_occupance_time,dt,t):
        for plane in self.locp:
            # plan operation
            self.plan_operation(self.type,ATC_list,runway_list,plane,graph,v_max,dt,t)
            # is aircraft at handoff point
            if sqrt((plane.x_pos - self.x_handoff)**2 + (plane.y_pos - self.y_handoff)**2) <= v_max*dt:
                self.create_commands(plane,ATC_list,runway_list,v_max,graph,runway_occupance_time,dt,t)
        return graph
        
    #create commands for each plane if necessary
    def create_commands(self,plane,ATC_list,runway_list,v_max,graph,runway_occupance_time,dt,t):
    # if I'm a runway/gate
        if self.type == 4:
            plane_atc = ATC_list[plane.atc[1]]
            runway = runway_list[plane_atc.par[0]['runway_id']]
            if plane not in runway.waiting_list:
                runway.waiting_list.append(plane)
            runway.take_off(plane,runway_occupance_time,ATC_list,graph)
#            runway.appendplane(plane,plane.atc[1],t)       # add to the correct runway
#            #check which runway the plane is at
#            for runway in runway_list: 
#                if self.id in runway.nodes:
#                    if not runway.occupance:
#                       self.remove_plane(plane)
#                    else:
#                        plane.stop = True
#                break
##########
#            occupance  = False #TODO runway systems needs to be implemented
#            # if runway available
#            if occupance == False:
#                # remove aircraft from sim
#                self.remove_plane(plane)
#                return
###########
        else:
#            self.plane_handoff(ATC_list,plane,t)
            if plane.stop:
#                print 'plane.stop executed'
                pass
            elif plane.op[0].par.has_key('next_atc') and (plane.op[0].par['next_atc'] == plane.atc[0]):
                print 'Plane', plane.id, ', at ATC ', self.id, ' made a 180 degree turn!'
                graph = self.plane_handoff(ATC_list,plane,graph,t)
            elif plane.op[0].par.has_key('next_atc') and (plane.op[0].par['next_atc'] != self.id):
                graph = self.plane_handoff(ATC_list,plane,graph,t)
            else:
                self.plan_operation(self.id,ATC_list,runway_list,plane,graph,v_max,dt,t)
                print 'Plane ' + str(plane.id) + ', at ATC ' + str(self.id) + 'needs a new ATC!'
        return graph

    def plan_operation(self,atc_type,ATC_list,runway_list,plane,graph,v_max,dt,t):
        par = {}
        command_type = 'heading'
        if atc_type == 1 or atc_type == 2:
            # run Dijkstra and see if solution possible
#            runway = runway_list[plane.atc_goal]
#            success = False
#            length = False
#            i = 0
#            for atc_id in runway.nodes: #check for the shortest route to the runway
#                success_dijk, path_dijk, length_dijk = dijkstra_path(graph,self.id,atc_id)
##                print 'at time ', t, ':'
##                print 'the length to atc ',atc_id,' is: ', length_dijk[atc_id]
##                print 'succes was: ', success_dijk
##                print 'path is: ', path_dijk
#                if i == 0:
#                    success = success_dijk
#                    path = path_dijk
#                    length = length_dijk[atc_id]
#                elif success_dijk and length_dijk[atc_id] < length:
#                    success = success_dijk
#                    length = length_dijk[atc_id]
#                    path = path_dijk
#                i = i + 1
            success, path = dijkstra_path(graph,self.id,plane.atc_goal)
            # if solution possible, command new heading/assign ATC
            if success:#path has been found, so aircraft doensn't have to stop
                plane.stop = False 
                # path = shortestpath(graph,self.id,plane.atc_goal) #give the current fastest route using Dijkstra algorithm
                next_atc = path[1] #selects the next atc
                new_heading = atan2((int(wp_database[int(next_atc)][2])-(self.y_handoff)), (int(wp_database[int(next_atc)][1])-(self.x_handoff))) #calculate the heading after the operation
                plane.heading = atan2((self.y_handoff-plane.y_pos), (self.x_handoff-plane.x_pos))
                turn_angle = new_heading - plane.heading # calculate the turn angle
                distance = hypot(plane.x_pos-self.x_handoff, plane.y_pos-self.y_handoff) #calculate at which distance the operation should be finished
                par['next_atc'] = next_atc
                par['turn_angle'] = turn_angle
                par['distance'] = distance
                plane_command = command(command_type, self.id, plane.id, t, 1, par) #1 = send
                plane.op.append(plane_command)
            # Otherwise tell the aircraft to stop immediately
            else: # no path has been found, so aircraft has to stop
#                print 'no path found'
                plane.stop = True
        if atc_type == 4:
            next_atc = self.id #selects the next atc
            new_heading = atan2((int(wp_database[int(next_atc)][2])-(self.y_handoff)), (int(wp_database[int(next_atc)][1])-(self.x_handoff))) #calculate the heading after the operation
            turn_angle = 0.5*pi # calculate the turn angle
            distance = hypot(plane.x_pos-self.x_handoff, plane.y_pos-self.y_handoff) #calculate at which distance the operation should be finished
            par['next_atc'] = next_atc
            par['turn_angle'] = turn_angle
            par['distance'] = distance
            plane_command = command(command_type, self.id, plane.id, t, 1, par) #1 = send
            plane.op.append(plane_command)
        
    def plane_handoff(self,ATC_list,plane,graph,t):
        source_atc = plane.atc[0]
        target_atc = plane.atc[1]
        next_atc = plane.op[0].par['next_atc']
        plane.op = []
        plane.par_avoid = {}
        plane.par_command = {}
        plane.update_atc(next_atc)
        plane.v_target = False
        plane.s_target = False
        plane.heading = False
        plane.heading_target = False
        plane.x_beg = float(wp_database[self.id][1])
        plane.y_beg = float(wp_database[self.id][2])
        plane.x_des = float(wp_database[next_atc][1])
        plane.y_des = float(wp_database[next_atc][2])

        # code to add a plane
        print 'Plane ' + str(plane.id) + ' added to link ' + str(target_atc) + '->' + str(next_atc)
        # update the density on the graph

        try:
            graph[target_atc][next_atc]['density'] += 1
            print 'Density now is ' + str(graph[target_atc][next_atc]['density'])
            if graph[target_atc][next_atc]['density'] > 0:
                print 'edge removed'
                graph.remove_edges_from([(next_atc,target_atc)])
                # graph[next_atc][target_atc]['density']= 100000000
            graph = ATC_list[next_atc].add_plane(plane,graph) # add to next ATC

            # code to remove the plane
            if source_atc and target_atc:
                print 'Plane ' + str(plane.id) + ' removed from link ' + str(source_atc) + '->' + str(target_atc)
                # update the density on the graph
                graph[source_atc][target_atc]['density'] -= 1
                # if the no more planes on this link, add link in the other direction
                if graph[source_atc][target_atc]['density'] == 0:
                    # graph[target_atc][source_atc]['density']=0
                    print 'edge added from ' + str(target_atc) + ' to ' + str(source_atc)
                    distance = hypot((wp_database[target_atc][1]-wp_database[source_atc][1]),(wp_database[target_atc][2]-wp_database[source_atc][2]))
                    value =  distance / 30*0.5144 #TODO replace 30 with v_max
                    graph.add_weighted_edges_from([(target_atc,source_atc,value)])
                    graph[target_atc][source_atc]['distance']=distance
                    graph[target_atc][source_atc]['density']=0
            graph = self.remove_plane(plane,graph) # remove from current ATC
        except:
            plane.stop = True
        return graph

    def remove_plane(self,plane,graph):
        self.locp.remove(plane)
        return graph

    def add_plane(self,plane,graph):
        self.locp.append(plane)
        return graph
                    
def create_ATC(wp_database,ATC_list):
    for i in xrange(len(wp_database)):
        ATC_linkdes1 = [elem for elem in wpl_database if int(elem[0]) == i] #makes a list of all links
        ATC_linkdes2 = [elem for elem in wpl_database if int(elem[1]) == i] #makes a list of all links
        ATC_link = ATC_linkdes1 + ATC_linkdes2  
        ATC_list.append(ATC(wp_database[i][0],ATC_link,[],int(wp_database[i][3]),float(wp_database[i][1]),float(wp_database[i][2])))
    return ATC_list