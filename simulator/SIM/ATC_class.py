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

    def update(self,ATC_list,runway_list,v_max,graph,dt,t):
        # check if commands for the aircraft are necessary
        self.command_check(ATC_list,runway_list,v_max,graph,dt,t)
    
    #check if a plane needs a command
    def command_check(self,ATC_list,runway_list,v_max,graph,dt,t):
        for plane in self.locp:
            self.create_commands(plane,ATC_list,runway_list,v_max,graph,dt,t)

    #create commands for each plane if necessary
    def create_commands(self,plane,ATC_list,runway_list,v_max,graph,dt,t):
        # plan operation
        self.plan_operation(self.type,ATC_list, plane, graph,t)
        # is aircraft at handoff point
        if sqrt((plane.x_pos - self.x_handoff)**2 + (plane.y_pos - self.y_handoff)**2) <= v_max*dt:
            # if I'm an runway/gate
            if self.type == 4:
                 occupance  = False
                 # if runway available
                 if occupance == False:
                     # remove aircraft from sim
                     self.remove_plane(plane)
                     return
            else:
                if plane.op[0].par.has_key('next_atc') and (plane.op[0].par['next_atc'] != self.id):
                    self.plane_handoff(ATC_list,plane,t) #TODO fix issue #16
                else:
                    print 'Plane ' + str(plane.id) + ', at ATC ' + str(self.id) + 'needs a new ATC!'
#         if plane.op == []:
#             self.plan_operation(self.type,ATC_list, plane, graph,t)
#             if self.type == 1:  #check to which type of ATC the aircraft is assigned
#                 self.plane_handoff(ATC_list,plane,t)
#             elif self.type == 4:
#                 if sqrt((plane.x_pos - self.x_handoff)**2 + (plane.y_pos - self.y_handoff)**2) <= v_max*dt:
# #                    occupance = runway_list[plane.atc_goal].occupance #check wether the runway is clear voor take-off
#                     occupance  = True
#                     if occupance == True:
#                         self.remove_plane(plane)
#         elif sqrt((plane.x_pos - self.x_handoff)**2 + (plane.y_pos - self.y_handoff)**2) <= v_max*dt: #check wether the aircraft is within range of its next destination ((v_max*dt))
#             self.plane_handoff(ATC_list,plane,t)

    def plan_operation(self,atc_type,ATC_list,plane,graph,t):
        par = {}
        command_type = 'heading'
        if atc_type == 1 or atc_type == 2:
            # run Dijkstra and see if solution possible
            success, path = dijkstra_path(graph,self.id,plane.atc_goal)
            # if solution possible, command new heading/assign ATC
            if success:
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
            else:
                print 'no path found'
                par['distance'] = 0.00000001
                par['v_target'] = 0
                plane_command = command('speed', self.id, plane.id, t, 1, par) #1 = send
                plane.op.append(plane_command)
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
        
    def plane_handoff(self,ATC_list,plane,t):
        next_atc = plane.op[0].par['next_atc'] #TODO fix issue #16
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
        ATC_list[next_atc].add_plane(plane) # add to next ATC
        self.remove_plane(plane) # remove from current ATC

    def remove_plane(self,plane):
        self.locp.remove(plane)

    def add_plane(self,plane):
        self.locp.append(plane)
    # excute commands and check for collision avoidence
    # def execute_command(self,separation,v_max,t,dt):
    #     for plane in self.locp:
    #         plane.decision_making(separation,v_max,dt)

def create_ATC(wp_database,ATC_list):
    for i in xrange(len(wp_database)):
        ATC_linkdes = [elem for elem in wpl_database if int(elem[0]) == i] #makes a list of all links away from this ATC
        ATC_linkd = [int(x[1]) for x in ATC_linkdes] # makes a list of all possible destination waypoints
        ATC_list.append(ATC(wp_database[i][0],ATC_linkd,[],int(wp_database[i][3]),float(wp_database[i][1]),float(wp_database[i][2])))
    return ATC_list
