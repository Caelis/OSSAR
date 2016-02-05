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
import time

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
        ### radar (distance of aircraft from ATC)
        self.update_radar()

        ### handoff decision && ### graph update
        self.handoff_decisions(ATC_list,graph,runway_list,runway_occupance_time)

        ### heading and speed commands
        self.plan_operations(graph,t)
        return graph

##############################
## RADAR
##############################
    def update_radar(self):
        for aircraft in self.locp:
            aircraft.handed_off = False
            self.calculate_aircraft_atc_distance(aircraft)

    def calculate_aircraft_atc_distance(self,aircraft):
        aircraft.distance_to_atc = hypot((self.x_handoff-aircraft.x_pos),(self.y_handoff-aircraft.y_pos))
        xAcRot,yAcRot = self.rotateXY(aircraft.heading,aircraft.x_pos,aircraft.y_pos)
        xAtcRot,yAtcRot = self.rotateXY(aircraft.heading,self.x_handoff,self.y_handoff)
        if yAcRot > yAtcRot:
            aircraft.distance_to_atc = -aircraft.distance_to_atc
            print('The distance is ' + str(aircraft.distance_to_atc) + '. We passed the ATC')

##############################
## Hand-off
##############################
    def handoff_decisions(self,ATC_list,graph,runway_list,runway_occupance_time):
        for aircraft in self.locp:
            if aircraft.distance_to_atc <= 0 and not aircraft.handed_off:
                aircraft.handed_off = True
                print 'Handoff aircraft ' + str(aircraft.id)
                self.plane_handoff(aircraft,ATC_list,graph,runway_list,runway_occupance_time)

    def plane_handoff(self,plane,ATC_list,graph,runway_list,runway_occupance_time):
        if self.type == 4:
            self.plane_handoff_runway(plane,ATC_list,graph,runway_list,runway_occupance_time)
        elif self.type == 1:
            self.plane_handoff_gate(plane,ATC_list,graph)
        elif self.type == 2:
            self.plane_handoff_intersection(plane,ATC_list,graph)
        else:
            'THIS IS NOT SUPPOSED TO HAPPEN! EACH NODE MUST HAVE A TYPE!!!'

    def plane_handoff_runway(self,plane,ATC_list,graph,runway_list,runway_occupance_time):
        plane_atc = ATC_list[plane.atc[1]]
        runway = runway_list[plane_atc.par[0]['runway_id']]
        if plane not in runway.waiting_list:
            runway.waiting_list.append(plane)
        runway.take_off(plane,runway_occupance_time,ATC_list,graph)

    def plane_handoff_intersection(self,plane,ATC_list,graph):
        ## Define the ATCs
        source_atc = plane.atc[0] # Where the plane is coming from
        target_atc = plane.atc[1] # Where the plane is currently going to
        next_atc = plane.op[0].par['next_atc'] # where the plane is going to next
        try:
            ## update graph density
            self.increase_graph_density(graph,target_atc,next_atc)
            self.decrease_graph_density(graph,source_atc,target_atc)
            ## process the aircraft handoff
            plane.process_handoff(next_atc,float(wp_database[self.id][1]), float(wp_database[self.id][2]), float(wp_database[next_atc][1]), float(wp_database[next_atc][2]))
            ## update plane -> atc assignment
            ATC_list[next_atc].add_plane(plane) # add to next ATC
            self.remove_plane(plane)
        except:
            plane.stop

    def plane_handoff_gate(self,plane,ATC_list,graph):
        success, path = self.get_path(graph,self.id,plane.atc_goal)
        if success:
            target_atc = plane.atc[1] # Where the plane is currently going to
            next_atc = path[1]

            ## update graph density
            self.increase_graph_density(graph,self.id,next_atc)

            ## update plane -> atc assignment
            # ATC_list[next_atc].calculate_aircraft_atc_distance(plane)
            ATC_list[next_atc].add_plane(plane) # add to next ATC
            self.remove_plane(plane)
            plane.process_handoff(next_atc,float(wp_database[target_atc][1]),float(wp_database[target_atc][2]),float(wp_database[next_atc][1]),float(wp_database[next_atc][2]))


        # Set values of plane for next link
        # plane.process_handoff(next_atc,float(wp_database[self.id][1]), float(wp_database[self.id][2]), float(wp_database[next_atc][1]), float(wp_database[next_atc][2]))

##############################
## Decision making
##############################
    def get_path(self,graph,origin,destination):
        success, path = dijkstra_path(graph,origin,destination)
        return success,path


##############################
## Supporting functions
##############################
    def rotateXY(self,angle,x,y): # Rotation matrix
        angle = -angle + pi/2
        xRot = cos(angle)*x - sin(angle)*y
        yRot = sin(angle)*x + cos(angle)*y
        return xRot,yRot

    def calculate_heading_change(self,plane,to_atc):
        if plane.heading:
            old_heading = plane.heading
        else:
            old_heading = 0

        if to_atc:
            new_heading = atan2((int(wp_database[int(to_atc)][2])-(self.y_handoff)), (int(wp_database[int(to_atc)][1])-(self.x_handoff))) #calculate the heading after the operation
        else:
            new_heading = 0
        return new_heading-old_heading

    def increase_graph_density(self,graph,source,target):
        graph[source][target]['density'] += 1
        print 'Density now is ' + str(graph[source][target]['density'])
        if graph[source][target]['density'] > 0:
            print 'edge removed'
            graph.remove_edges_from([(target,source)])

    def decrease_graph_density(self,graph,source,target):
        # print 'Plane ' + str(plane.id) + ' removed from link ' + str(source_atc) + '->' + str(target_atc)
        # update the density on the graph
        graph[source][target]['density'] -= 1
        # if the no more planes on this link, add link in the other direction
        if graph[source][target]['density'] == 0:
            print 'edge added from ' + str(target) + ' to ' + str(source)
            distance = hypot((wp_database[target][1]-wp_database[source][1]),(wp_database[target][2]-wp_database[source][2]))
            value =  distance / 30*0.5144 #TODO replace 30 with v_max
            graph.add_weighted_edges_from([(target,source,value)])
            graph[target][source]['distance']=distance
            graph[target][source]['density']=0

##############################
## Command functions
##############################
    #check if a plane needs a command
    def plan_operations(self,graph,t):
        for plane in self.locp:
            # plan operation
            self.plan_operation(plane,graph,t)

    def plan_operation(self,plane,graph,t):
        par = {}
        atc_type = self.type
        command_type = 'heading'
        if atc_type == 1 or atc_type == 2:
            success, path = self.get_path(graph,self.id,plane.atc_goal)
            # if solution possible, command new heading/assign ATC
            if success:#path has been found, so aircraft doensn't have to stop
                plane.stop = False 
                next_atc = path[1] #selects the next atc

                # calculate new heading and distance
                turn_angle = self.calculate_heading_change(plane,next_atc)
                distance = hypot(plane.x_pos-self.x_handoff, plane.y_pos-self.y_handoff) #calculate at which distance the operation should be finished

                # parameters of command
                par['next_atc'] = next_atc
                par['turn_angle'] = turn_angle
                par['distance'] = distance

                # send command to plane
                plane_command = command(command_type, self.id, plane.id, t, 1, par) #1 = send
                plane.op.append(plane_command)
            # Otherwise tell the aircraft to stop immediately
            else: # no path has been found, so aircraft has to stop
#                print 'no path found'
                plane.stop = True
        if atc_type == 4:
            next_atc = self.id #selects the next atc
            turn_angle = 0.5*pi # calculate the turn angle
            distance = hypot(plane.x_pos-self.x_handoff, plane.y_pos-self.y_handoff) #calculate at which distance the operation should be finished
            par['next_atc'] = next_atc
            par['turn_angle'] = turn_angle
            par['distance'] = distance
            plane_command = command(command_type, self.id, plane.id, t, 1, par) #1 = send
            plane.op.append(plane_command)

    def remove_plane(self,plane):
        self.locp.remove(plane)

    def add_plane(self,plane):
        self.locp.append(plane)
                    
def create_ATC(wp_database,ATC_list):
    for i in xrange(len(wp_database)):
        ATC_linkdes1 = [elem for elem in wpl_database if int(elem[0]) == i] #makes a list of all links
        ATC_linkdes2 = [elem for elem in wpl_database if int(elem[1]) == i] #makes a list of all links
        ATC_link = ATC_linkdes1 + ATC_linkdes2  
        ATC_list.append(ATC(wp_database[i][0],ATC_link,[],int(wp_database[i][3]),float(wp_database[i][1]),float(wp_database[i][2])))
    return ATC_list