'''
Package: main
Module: aircraft

description: This module makes it possible to create aircraft as instances

Input:
Output:

Package dependence:
'''

#Import python modules
import numpy as np
import random
from math import *
from data_import import data
from data_import import wp_database

# Aircraft class is created
class aircraft:

    # Properties of the aircraft 
    def __init__(self,Number,Type,Speed,max_speed,max_acceleration,max_decceleration,X0,Y0,X1,Y1,X2,Y2,heading,ATC_id,ATC_runway):
        self.type = int(Type)
        self.id = Number            #identification number of the aircraft
        self.v = Speed              #current speed of the aircraft
        self.v_limit = max_speed    #maximum speed the aircaft is able to achieve due to direct circumstances
        self.v_target = False       #target speed due to commands
        self.heading = heading      #current heading of the aircraft
        self.heading_target = False #target heading due to commands
        self.s_target = False       #distance at which the aircraft needs to start its operation
        self.acc = max_acceleration
        self.dcc = max_decceleration
        self.x_beg = X0
        self.y_beg = Y0
        self.x_pos = X1
        self.y_pos = Y1
        self.x_des = X2             #current x_coordinate of destination
        self.y_des = Y2             #current y_coordinate of destination 
        self.flightplan = []
        self.col_list = []
        self.atc = ATC_id
        self.atc_goal = ATC_runway
        self.op = []

    # Update positions using velocity and heading (heading is obtained from ATC)
    def update_pos(self,dt):                                           
        if self.heading == False or self.heading == None: #if no heading given, determine heading
            self.change_heading()
        self.x_pos = self.x_pos + self.v * dt * cos(self.heading)
        self.y_pos = self.y_pos + self.v * dt * sin(self.heading)
#        print self.v_target

#    #check if there are any aircraft at or within seperation minimum and if so, change the speed limit
#    def collision_avoidence(self,ATC_list): ### not in use yet
#        for plane in ATC_list[self.atc].locp:
#            planecheck #if planes within serperation distance v_limit = speed of the leading aircraft
#        self.v_limit = v_limit
#    
#    #check if there are new commands given
#    def check_newcommands(self,v_max,dt): ### not in use yet
#        for command in self.op:
#            if command.status == 1:
#                command.status = 2
#                self.set_target(self.op[0].type,v_max,dt)
#                self.check_command(v_max,dt) #check if the command is executable
#    
#    def check_command(self,v_max,dt): ### not in use yet
#        v_target, s_target = self.set_target(command,v_max,dt)
#        if v_target >= self.v_limit:
#            self.v_target = self.v_limit
#        else:
#            self.v_target = v_target

#    def set_target(self,command,v_max,dt): #plan the operation  ### not in use yet
#        if command.type == 1: #turn command
#            if command.value < 0.03*pi: #if turn angle is smaller then 5 degrees
#                v_target = v_max
#                s_target = 0
#            elif command.value > 0.47*pi and command.value < 0.53*pi or command.value < -0.47*pi and command.value > -0.53*pi: #if turn angle is smaller then 5 degrees
#                v_target = 0.5144 * int(data[self.type][2]) #set target turn speed
#                dcc_dist = 0.5*(v_target-self.v)**2/self.dcc + self.v*abs(v_target-self.v)/self.dcc #calculate the distance needed to deccelerate
#                tot_dist = command.distance #total distance until next atc
#                s_target = tot_dist-dcc_dist #the operation starts when the plane's current distance equals the atcdistanve minus the operation distance
#            else:
#                v_target = v_max
#                s_target = 0
#        if command.type == 2: #speed command
#            v_target = command.value
#        return v_target, s_target

    #check if there are new commands given
    def check_newcommands(self,v_max,dt): ### not in use yet
        self.op[0].status = 2
        self.set_target(self.op[0].type,v_max,dt)
    
    def execute(self,v_max,dt): #execute commands given to this aircraft this timestep
        if self.op == []:
            self.heading = self.change_heading()
        elif self.op[0].status == 1:
            self.check_newcommands(v_max,dt)
        elif self.op[0].status == 2:
            if self.op[0].type == 1: #turn command
                if self.s_target < v_max*dt: #check if the operation can be started
#                    print "distance to target is: " +str(self.s_target)
#                    print "marge is: " +str(v_max*dt)
                    if self.v != self.v_target:
                        self.change_speed(v_max,dt)
                else:
                    self.s_target = self.s_target - self.v*dt
                    self.change_speed(v_max,dt)
            elif self.op[0].type == 2: #speed command
                if self.v != self.v_target:
                    self.change_speed(v_max,dt)

    def set_target(self,op_type,v_max,dt): #plan the operation
        if op_type == 1: #turn command
            if -0.03*pi < self.op[0].value < 0.03*pi or  0.97*pi < self.op[0].value < 1.01*pi or -0.97*pi > self.op[0].value > -1.01*pi: #if turn angle is smaller then 5 degrees
                self.v_target = v_max
                self.s_target = 0
            elif 0.47*pi < self.op[0].value < 0.53*pi or -0.47*pi > self.op[0].value > -0.53*pi: #if turn angle is smaller then 5 degrees
                self.v_target = 0.5144 * int(data[self.type][2]) #set target turn speed
                dcc_dist = 0.5*(self.v_target-self.v)**2/self.dcc + self.v*abs(self.v_target-self.v)/self.dcc #calculate the distance needed to deccelerate
                tot_dist = self.op[0].distance #total distance until next atc
                self.s_target = tot_dist-dcc_dist #the operation starts when the plane's current distance equals the atcdistanve minus the operation distance
            else:
                self.v_target = v_limit
                self.s_target = 0
        if self.op[0].type == 2: #speed command
            self.v_target = self.op[0].value
            
    def change_speed(self,v_max,dt):
        if self.v_target == False:
            if self.v != v_max:
                new_speed = self.v + dt * (float(data[self.type][4])*0.51444)
        elif self.v < self.v_target:
            new_speed = self.v + dt * (float(data[self.type][4])*0.51444)
            if new_speed > self.v_target:
                self.v = self.v_target
            else:
                self.v = new_speed
        elif self.v > self.v_target:
            new_speed = self.v - dt * (float(data[self.type][3])*0.51444)
            if new_speed < self.v_target:
                self.v = self.v_target
            else:
                self.v = new_speed

    def change_heading(self):
        self.heading = atan2((self.y_des-self.y_pos), (self.x_des-self.x_pos))

        
    def change_atc(self,new_atc):
        self.atc = new_atc
        
    # Update position in list 
#    def update_listnumber(self, planes):                                             
#        listid = []
#        for i in range(len(planes)):
#            listid.append(planes[i].id)
#        self.listnumber = listid.index(self.id)