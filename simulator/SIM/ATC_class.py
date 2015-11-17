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

#import OSSAR modules
from Dijkstra_class import shortestpath
from command_class import *

#import data
from data_import import wp_database
#from collision_class import collision

##create instances
#coll_ins = collision()

class ATC:
    
    def __init__(self, ATC_ID, ATClink, locplanes, ATCtype, x_handoff, y_handoff):
        self.id = ATC_ID                #ATC identification number
        self.link = ATClink             #Array of possible handover ATC
        self.locp = locplanes           #Array of planes under command
        self.type = ATCtype             #Type of ATC (gate(1), waypoint(2), end(4))
        self.x_handoff = x_handoff      #x-coordinate at which a aircraft should be handed over
        self.y_handoff = y_handoff      #y-coordinate at which a aircraft should be handed over
        
    def create_commands(self, ATC_list,v_max,structure,dt,t):
        for plane in self.locp:  # check if each plane has a command
            if plane.op == []:
                if self.type == 1:  #check to which type of ATC the aircraft is assigned
                    self.plan_operation(self.type,ATC_list, plane,structure,t)
                    self.plane_handoff(ATC_list,plane,t)
                elif self.type == 2:
                    self.plan_operation(self.type,ATC_list, plane,structure,t)
                elif self.type == 4:
                    self.plan_operation(self.type,ATC_list, plane,structure,t)
                    if sqrt((plane.x_pos - self.x_handoff)**2 + (plane.y_pos - self.y_handoff)**2) <= v_max*dt:
                        self.remove_plane(plane)
            elif sqrt((plane.x_pos - self.x_handoff)**2 + (plane.y_pos - self.y_handoff)**2) <= v_max*dt: #check wether the aircraft is within range of its next destination ((v_max*dt))
                self.plane_handoff(ATC_list,plane,t)

    def plan_operation(self,atc_type,ATC_list,plane,structure,t):
        # turn  = type 1 operation (o to 2 pi radian)
        # change speed = type 2 operation
        if atc_type == 1 or atc_type == 2:
            path = shortestpath(structure,self.id,plane.atc_goal) #give the current fastest route using Dijkstra algorithm
            next_atc = path[1] #selects the next atc
            new_heading = atan2((int(wp_database[int(next_atc)][2])-(self.y_handoff)), (int(wp_database[int(next_atc)][1])-(self.x_handoff))) #calculate the heading after the operation
            plane.heading = atan2((self.y_handoff-plane.y_pos), (self.x_handoff-plane.x_pos))
            turn_angle = new_heading - plane.heading # calculate the turn angle
            distance = hypot(plane.x_pos-self.x_handoff, plane.y_pos-self.y_handoff) #calculate at which distance the operation should be finished
            command_type = 1
            plane_command = command(command_type, distance, turn_angle, self.id, plane.id, t ,next_atc, 1) #1 = send
        if atc_type == 4:      
            next_atc = self.id #selects the next atc
            new_heading = atan2((int(wp_database[int(next_atc)][2])-(self.y_handoff)), (int(wp_database[int(next_atc)][1])-(self.x_handoff))) #calculate the heading after the operation
            turn_angle = 0.5*pi # calculate the turn angle
            distance = hypot(plane.x_pos-self.x_handoff, plane.y_pos-self.y_handoff) #calculate at which distance the operation should be finished
            command_type = 1
            plane_command = command(command_type, distance, turn_angle, self.id, plane.id, t ,next_atc, 1) #1 = send     
        plane.op.append(plane_command)
        
    def plane_handoff(self,ATC_list,plane,t):
        next_atc = plane.op[0].next_atc
        plane.op = []
        plane.update_atc(next_atc)
        plane.v_target = False
        plane.s_target = False
        plane.heading = False
        plane.heading_target = False
        plane.x_des = float(wp_database[next_atc][1])
        plane.y_des = float(wp_database[next_atc][2])
        ATC_list[next_atc].add_plane(plane) # add to next ATC
        self.remove_plane(plane) # remove from current ATC

    def remove_plane(self,plane):
        self.locp.remove(plane)

    def add_plane(self,plane):
        self.locp.append(plane)