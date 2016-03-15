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
from dijkstra_structure import *
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
        self.runway = False             # for a runway ATC the runway that it has associated
        self.num_links = len(self.link)

    # def update_aircraft_distance(self):
    #     for thisPlane in self.locp:
    #         thisPlane.


    # check if commands for the plane are necessary and update planned operation
    def update(self,ATC_list,runway_list,v_max,graphDict,runway_occupance_time,dt,t,simulation_constants):
        graph = graphDict['graph']
        ### radar (distance of aircraft from ATC)
        self.update_radar()

        self.check_and_remove_impossible_conflict(graphDict,simulation_constants)

        ### handoff decision && ### graph update
        planes_taxi_time = self.handoff_decisions(ATC_list,graphDict,runway_list,runway_occupance_time,t,simulation_constants)

        ### heading and speed commands
        self.plan_operations(graphDict,t)
        return graph, planes_taxi_time

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
            # print('The distance is ' + str(aircraft.distance_to_atc) + '. We passed the ATC')

    def check_and_remove_impossible_conflict(self, graphDict,simulation_constants):
        if self.type == 2:
            # with this function we check if there are unsolvable conflcits and reomove all aircraft that are part of the conflict.
            closest_incoming = {}
            # make a dictionary of the closest aircraft
            for aircraft in self.locp:
                if closest_incoming.has_key(aircraft.atc[0]):
                    if closest_incoming[aircraft.atc[0]].distance_to_atc > aircraft.distance_to_atc:
                        closest_incoming[aircraft.atc[0]] = aircraft
                else:
                    closest_incoming[aircraft.atc[0]] = aircraft
            if len(closest_incoming) == self.num_links:
                for link_id in closest_incoming:
                    aircraft = closest_incoming[link_id]
                    # print aircraft
                    aircraft.stop = aircraft.stop | 256
                    self.remove_plane(aircraft)
                    self.decrease_graph_density(graphDict, aircraft.atc[0], self.id,simulation_constants)
                    aircraft.is_active = False

##############################
## pre Hand-off
##############################
    # def pre_handoff_decisions(self,graph):
    #     for aircraft in self.locp:
    #         if not aircraft.check_if_ac_can_stop(aircraft.distance_to_atc,'comfort'):
    #             self.pre_handoff_decision(aircraft,graph)

    def pre_handoff_type_select(self,aircraft,graphDict,simulation_constants):
        if self.type == 4:
            # print 'Handoff at Runway'
            self.pre_handoff_runway(aircraft,graphDict)
        elif self.type == 1:
            self.pre_handoff_gate(aircraft,graphDict)
        elif self.type == 2:
            self.pre_handoff_intersection(aircraft,graphDict,simulation_constants)
        else:
            'THIS IS NOT SUPPOSED TO HAPPEN! EACH NODE MUST HAVE A TYPE!!!'

    def pre_handoff_intersection(self,plane,graphDict,simulation_constants):
        graph = graphDict['graph']
        if len(plane.op) > 0:
            plane.stop = plane.stop ^ (plane.stop & 2)
            if plane.op[-1].par.has_key('next_atc'):
                target_atc = plane.atc[1] # Where the plane is currently going to
                next_atc = plane.op[-1].par['next_atc'] # where the plane is going to next
                if graph.has_edge(target_atc,next_atc):
                    plane.ready_for_hand_off = True
                    self.increase_graph_density(graphDict,target_atc,next_atc,simulation_constants)
                    plane.stop = plane.stop ^ (plane.stop & 2)
                    # print 'Pre Handoff successfull Plane ' + str(plane.id) + ' ATC: ' + str(self.id)
                else:
                    plane.stop = plane.stop | 2
                    # print 'Emergency stop Plane ' + str(plane.id) + ' ATC: ' + str(self.id) + ' code: ' + str(plane.stop) + ' to: ' + str(plane.atc_goal)

            else:
                plane.stop = plane.stop | 2
        else:
            plane.stop = plane.stop | 2

    def pre_handoff_gate(self,plane,graphDict):
        plane.stop = plane.stop ^ (plane.stop & 32)
        plane.ready_for_hand_off = True

    def pre_handoff_runway(self,plane,graph):
        # print 'Check pre-handoff for plane ',plane.id
        # print 'The handoff-status is: ',plane.ready_for_hand_off
        # check if aircraft is in Waiting list
        if not plane.id in self.runway.waiting_list:
            self.runway.waiting_list.append(plane.id)
            # print self.runway.waiting_list[0]
        # check if the runwya is available for a departure
        if self.runway.is_occupied():
            plane.stop = plane.stop | 16
        else:
            if self.runway.waiting_list[0] == plane.id:
                # print 'Check worked'
                plane.stop = plane.stop ^ (plane.stop & 16)
                plane.ready_for_hand_off = True
                self.runway.reset_occupance()


##############################
## Hand-off
##############################

    def handoff_decisions(self,ATC_list,graphDict,runway_list,runway_occupance_time,t,simulation_constants):
        planes_taxi_time = []
        for aircraft in self.locp:
            # Pre-handoff decision
            if ((not aircraft.check_if_ac_can_stop(aircraft.distance_to_atc,'comfort')) or aircraft.stop & 16 or aircraft.stop & 2) and not aircraft.ready_for_hand_off:
                self.pre_handoff_type_select(aircraft,graphDict,simulation_constants)
            # Handoff decision
            if aircraft.distance_to_atc <= 0 and not aircraft.handed_off and aircraft.ready_for_hand_off:
                aircraft.ready_for_hand_off = False
                aircraft.handed_off = True
                # aircraft.stop = False
                # print 'Handoff aircraft ' + str(aircraft.id)

                plane_taxi_time = self.plane_handoff(aircraft,ATC_list,graphDict,runway_list,runway_occupance_time,t,simulation_constants)
                if plane_taxi_time: # plane_taxi_time is only True if the aircraft was handed of to a runway
                    planes_taxi_time.append(plane_taxi_time)
        return planes_taxi_time

    def plane_handoff(self,plane,ATC_list,graphDict,runway_list,runway_occupance_time,t,simulation_constants):
        plane_taxi_time = False
        if self.type == 4:
            plane_taxi_time = self.plane_handoff_runway(plane,ATC_list,graphDict,runway_list,runway_occupance_time,t,simulation_constants)
        elif self.type == 1:
            self.plane_handoff_gate(plane,ATC_list,graphDict,simulation_constants)
        elif self.type == 2:
            self.plane_handoff_intersection(plane,ATC_list,graphDict,simulation_constants)
        else:
            'THIS IS NOT SUPPOSED TO HAPPEN! EACH NODE MUST HAVE A TYPE!!!'
        return plane_taxi_time

    def plane_handoff_runway(self,plane,ATC_list,graphDict,runway_list,runway_occupance_time,t,simulation_constants):
        source_atc = plane.atc[0] # Where the plane is coming from
        target_atc = plane.atc[1] # Where the plane is currently going to
        # make sure that AC in waiting list of runway
        if plane.id in self.runway.waiting_list:
            # if the runwya is available
            # print 'occupance is:',self.runway.occupance
            # if not self.runway.occupance > 0:
            plane.stop = plane.stop ^ (plane.stop & 16)
            # plane = self.waiting_list[0]
            self.runway.reset_occupance()       # reset occupancy time
            # ATC_list[plane.atc[1]].remove_plane(plane)  # remove plane from ATC
            self.runway.waiting_list.remove(plane.id)         # remove plane from waiting_list
            self.decrease_graph_density(graphDict,source_atc,target_atc,simulation_constants)
            plane.is_active = False

            #calculate the taxi time
            plane_taxi_time = t - plane.spawntime

            #remove the aircraft from the simulator
            self.remove_plane(plane)
        else:
            print 'BIG problem!!!'
        return plane_taxi_time

    def plane_handoff_intersection(self,plane,ATC_list,graphDict,simulation_constants):
        graph = graphDict['graph']
        ## Define the ATCs
        source_atc = plane.atc[0] # Where the plane is coming from
        target_atc = plane.atc[1] # Where the plane is currently going to
        next_atc = plane.op[-1].par['next_atc'] # where the plane is going to next
        if graph.has_edge(source_atc,target_atc):
            plane.stop = plane.stop ^ (plane.stop & 1)
            ## update graph density
            self.decrease_graph_density(graphDict,source_atc,target_atc,simulation_constants)
            ## process the aircraft handoff
            plane.process_handoff(next_atc,float(wp_database[self.id][1]), float(wp_database[self.id][2]), float(wp_database[next_atc][1]), float(wp_database[next_atc][2]))
            ## update plane -> atc assignment
            ATC_list[next_atc].add_plane(plane) # add to next ATC
            self.remove_plane(plane)
        else:
            plane.stop = plane.stop | 1

    def plane_handoff_gate(self,plane,ATC_list,graphDict,simulation_constants):
        graph = graphDict['graph']
        target_atc = plane.atc[1] # Where the plane is currently going to
        success, path = self.get_path(graph, self.id, plane.atc_goal,plane.route)
        if success:
            next_atc = path[1]
        else:
            next_atc = self.link[0][1]

        ## update graph density
        self.increase_graph_density(graphDict,self.id,next_atc,simulation_constants)

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
    def get_path(self,graph,origin,destination,route):
        success, path = dijkstra_path(graph,origin,destination)
        if not success:
            if not isinstance(route, list):
                print '####### This is not a list:',route
                route = [route]
            if len(route)>0:
                destination = route[-1]
                cutRoute = route[0:-1]
                # if not origin == destination:
                success, path = self.get_path(graph,origin,destination,cutRoute)
                # elif not len(route)>1:
                #     print '### The route was too short:',len(route)
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


##############################
## Graph handling functions
##############################
    def increase_graph_density(self,graphDict,source,target,simulation_constants):
        self.increase_graph_density_one_dict(graphDict,source,target,simulation_constants)

    def increase_graph_density_one_dict(self,graphDict,source,target,simulation_constants):
        graph = graphDict['graph']
        dummyGraph = graphDict['dummyGraph']
        if graph[source][target]['density'] == 0:
            # first remove linked edges
            for edge in graph[target][source]['linked_edges']:
                dummyGraph.remove_edges_from([(edge[0],edge[1])])
                # print 'removed the edge',edge,':',dummyGraph[edge[0]]
            # print 'edge removed'
            graph.remove_edges_from([(target,source)])
        graph[source][target]['density'] += 1
        calculate_edge_values(graph[source][target],simulation_constants)
        #also update density of linked edges
        for edge in graph[source][target]['linked_edges']:
            # print 'Edge:',edge
            # print 'From:',dummyGraph[edge[0]]
            # print 'From/to:',dummyGraph[edge[0]][edge[1]]
            dummyGraph[edge[0]][edge[1]]['density'] = graph[source][target]['density']
            calculate_edge_values(dummyGraph[edge[0]][edge[1]],simulation_constants)
        # print 'Density now is ' + str(graph[source][target]['density'])

    def decrease_graph_density(self,graphDict,source,target,simulation_constants):
        self.decrease_graph_density_one_dict(graphDict,source,target,simulation_constants)

    def decrease_graph_density_one_dict(self,graphDict,source,target,simulation_constants):
        graph = graphDict['graph']
        dummyGraph = graphDict['dummyGraph']
        # print 'removed from link ' + str(source) + '->' + str(target)
        # update the density on the graph
        graph[source][target]['density'] -= 1
        calculate_edge_values(graph[source][target],simulation_constants)
        for edge in graph[source][target]['linked_edges']:
            dummyGraph[edge[0]][edge[1]]['density'] = graph[source][target]['density']
            calculate_edge_values(dummyGraph[edge[0]][edge[1]],simulation_constants)

        # if the no more planes on this link, add link in the other direction
        if graph[source][target]['density'] == 0:
            # print 'edge added from ' + str(target) + ' to ' + str(source)
            self.add_edge_back_to_graph(graphDict,target,source)

    def calculate_edge_values(self,edge,simulation_constants):
        # get simulation constants
        separation = simulation_constants['separation']
        v_max = simulation_constants['v_max']
        v_turn = simulation_constants['v_turn']
        dec_standard = simulation_constants['dec_standard']
        acc_standard = simulation_constants['acc_standard']
        flowTheory_cutoff = simulation_constants['flowTheory_cutoff']

        # get static edge values
        density = edge['density']
        distance = edge['distance']

        # calculate dynamic edge values
        max_density = distance / separation

        ## Calculate the speeds based on density
        if density > max_density:
            speed = 0
        elif density < flowTheory_cutoff * max_density: # 0.5 * max_density:  # TODO make the the 0.5 a variable that is set up in the simulator setup
            speed = v_max
        else:
            if density > 0:
                speed = v_max * (1 - (density / max_density))
            else:
                speed = v_max

        # Calculate the (graph weight = time_value) based on the speed and distance
        if speed > 0:
            time_value = distance / speed  # TODO this is NOT our optimal soultion!  BALANCING DILEMMA
        else:
            time_value = float('Inf')

        time_for_link = time_with_turn_penalty(distance,speed,v_turn,dec_standard,acc_standard,edge['turn'])

        edge['weight'] = time_for_link
        edge['speed'] = speed

    def add_edge_back_to_graph(self,graphDict,source,target):
        graph = graphDict['graph']
        graphOrig = graphDict['graphOrig']
        dummyGraph = graphDict['dummyGraph']
        dummyGraphOrig = graphDict['dummyGraphOrig']
        graph.add_edge(source,target)
        graph[source][target] = graphOrig[source][target]
        for edge in graph[source][target]['linked_edges']:
            dummyGraph.add_edge(edge[0],edge[1])
            dummyGraph[edge[0]][edge[1]] = dummyGraphOrig[edge[0]][edge[1]]


##############################
## Command functions
##############################
    #check if a plane needs a command
    def plan_operations(self,graphDict,t):
        for plane in self.locp:
            if not plane.ready_for_hand_off: # once the aircraft has crossed the "pre handoff threshold", don't change the operations any more
                self.plan_operation(plane,graphDict,t)

    def plan_operation(self, plane, graphDict, t):
        tempGraph = graphDict['dummyGraph']
        # tempGraph.remove_edges_from([(plane.atc[1],plane.atc[0])])
        if self.type == 1:
            self.plan_operation_gate(tempGraph, plane, t)
        elif self.type == 2:
            self.plan_operation_intersection(tempGraph, plane, t)
        elif self.type == 4:
            self.plan_operation_runway(plane,t)
        else:
            'THIS IS NOT SUPPOSED TO HAPPEN! EACH PLANE MUST HAVE AN ATC!!!'

    def plan_operation_gate(self,graph,plane,t):
        # success, path = self.get_path(graph,self.id,plane.atc_goal,plane.route)
        success, path = self.get_path(graph,str(self.id),str(plane.atc_goal),plane.route)
        if success:
            plane.stop = plane.stop ^ (plane.stop & 8)
            next_atc = int(path[1].split('_')[0]) #selects the next atc
            # calculate new heading
            turn_angle = self.calculate_heading_change(plane,next_atc)
            self.set_operation_parameters(plane,next_atc,turn_angle,t)
            # self.send_route_to_plane(plane,path,t)
        else:
            plane.stop = plane.stop | 8
    
    def plan_operation_intersection(self,graph,plane,t):
        # success, path = self.get_path(graph,self.id,plane.atc_goal,plane.route)
        success, path = self.get_path(graph,str(self.id),str(plane.atc_goal),plane.route)
        if success and len(path)>1:
            plane.stop = plane.stop ^ (plane.stop & 8)
            next_atc = int(path[1].split('_')[0]) #selects the next atc
            # calculate new heading
            turn_angle = self.calculate_heading_change(plane,next_atc) 
            self.set_operation_parameters(plane,next_atc,turn_angle,t)
            self.send_route_to_plane(plane,path,t)
        else:
            plane.stop = plane.stop | 8

    def plan_operation_runway(self,plane,t):
        next_atc = self.id #selects the next atc
        turn_angle = 0.5*pi # calculate the turn angle
        self.set_operation_parameters(plane,next_atc,turn_angle,t)

    def set_operation_parameters(self,plane,next_atc,turn_angle,t):
        par = {}
        command_type = 'heading'
        distance = plane.distance_to_atc #hypot(plane.x_pos-self.x_handoff, plane.y_pos-self.y_handoff) #calculate at which distance the operation should be finished
        # parameters of command
        par['next_atc'] = next_atc
        par['turn_angle'] = turn_angle
        par['distance'] = distance
        # send command to plane
        plane_command = command(command_type, self.id, plane.id, t, 1, par) #1 = send
        plane.op.append(plane_command)

    def send_route_to_plane(self,plane,route,t):
        par = {}
        command_type = 'route'
        par['route'] = route
        # send command to plane
        plane_command = command(command_type, self.id, plane.id, t, 1, par) #1 = send
        plane.op.append(plane_command)

    def remove_plane(self,plane):
        self.locp.remove(plane)

    def add_plane(self,plane):
        self.locp.append(plane)

##############################
## Creating all ATC
##############################
                    
def create_ATC(wp_database,ATC_list):
    for i in xrange(len(wp_database)):
        ATC_linkdes1 = [elem for elem in wpl_database if int(elem[0]) == i] #makes a list of all links
        ATC_linkdes2 = [elem for elem in wpl_database if int(elem[1]) == i] #makes a list of all links
        ATC_link = ATC_linkdes1 + ATC_linkdes2  
        ATC_list.append(ATC(wp_database[i][0],ATC_link,[],int(wp_database[i][3]),float(wp_database[i][1]),float(wp_database[i][2])))
    return ATC_list