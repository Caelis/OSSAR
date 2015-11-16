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
    def __init__(self,Type,Speed,max_acceleration,max_decceleration,X0,Y0,X1,Y1,X2,Y2,heading,ATC_id,ATC_runway):
        self.type = int(Type)
        self.v = Speed
        self.acc = max_acceleration
        self.dcc = max_decceleration
        self.v_target = False
        self.s_target = False #distance at which the aircraft needs to start its operation
        self.x_beg = X0
        self.y_beg = Y0
        self.x_pos = X1
        self.y_pos = Y1
        self.x_des = X2 #goal x_coordinate
        self.y_des = Y2 #goal y_coordinate
#        self.listnumber = listnumber
#        self.id = Number
        self.heading = heading
        self.flightplan = []
        self.col_list = []
        self.atc = ATC_id
        self.atc_goal = ATC_runway
        self.op = []

    # Update positions using velocity and heading (heading is obtained from ATC)
    def update_pos(self,dt):                                           
        if self.heading == False or self.heading == None:
            self.change_heading()
        self.x_pos = self.x_pos + self.v * dt * cos(self.heading)
        self.y_pos = self.y_pos + self.v * dt * sin(self.heading)
#        print "current operation is: " + str(self.op)
#        print "speed = " + str(self.v)
#        print "target speed = " + str(self.v_target)

    # Update position in list 
    def update_listnumber(self, planes):                                             
        listid = []
        for i in range(len(planes)):
            listid.append(planes[i].id)
        self.listnumber = listid.index(self.id)
        
    def execute(self,v_max,dt): #execute commands given to this aircraft this timestep
        if self.op == []:
            self.change_speed(v_max,dt)
            self.heading = self.change_heading()
        elif self.op[0] == 1: #turn command
            if self.v_target == False or self.s_target == False: #check if the operation needs to be planned
                self.set_target(self.op[0],v_max,dt)
            if self.s_target < v_max*dt: #check if the operation can be started
#                print "target distance is achieved"
                if self.v != self.v_target:
                    self.change_speed(v_max,dt)
            else:
                self.s_target = self.s_target - self.v*dt
        elif self.op[0] == 2: #speed command
            if self.v_target == False: #check wether the operation needs to be planned
                self.set_target(self.op[0],v_max,dt)
            elif self.v != self.v_target:
                self.change_speed(v_max,dt)

    def set_target(self,op_type,v_max,dt): #plan the operation
#        print "target is set########################################"
        if op_type == 1: #turn command
            if self.op[2] < 0.03*pi: #if turn angle is smaller then 5 degrees
                self.v_target = v_max
                self.s_target = 0
            elif self.op[2] > 0.47*pi and self.op[2] < 0.53*pi or self.op[2] < -0.47*pi and self.op[2] > -0.53*pi: #if turn angle is smaller then 5 degrees
                self.v_target = 0.5144 * int(data[self.type][2]) #set target turn speed
                dcc_dist = 0.5*(self.v_target-self.v)**2/self.dcc + self.v*abs(self.v_target-self.v)/self.dcc #calculate the distance needed to deccelerate
                tot_dist = self.op[1] #total distance until next atc
                self.s_target = tot_dist-dcc_dist #the operation starts when the plane's current distance equals the atcdistanve minus the operation distance
            else:
                self.v_target = v_max
                self.s_target = 0
        if self.op[0] == 2: #speed command
            self.v_target = self.op[2]
            
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