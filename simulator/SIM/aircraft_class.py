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
        self.atc = [False, ATC_id]  #plane goes from self.atc[0] to self.atc[1]
        self.atc_goal = ATC_runway
        self.op = []

    def decision_making(self,ATC_list,separation,v_max,dt):
        self.collision_avoidence(ATC_list,separation,v_max)    # check if there are any aircraft within seperation minimum
        self.check_newcommands(v_max,dt)            # check if new commands were given
        if len(self.op) != 0:
            self.execute(v_max,dt)                      # execute given commands
        else:
            self.check_speed(v_max,dt)
        
    #check if there are any aircraft at or within seperation minimum and if so, change the speed limit
    def collision_avoidence(self,ATC_list,separation,v_max):
        min_seperation = separation #minimal seperation [m] for taxiing aircraft
        other_plane = [elem for elem in ATC_list[self.atc[1]].locp if elem.id != self.id ]
        if len(other_plane) != 0:
            for plane in other_plane:
                plane_seperation = hypot((self.x_pos-plane.x_pos), (self.y_pos-plane.y_pos))
                if plane_seperation < min_seperation: #check if seperation is lost
                    heading_diff = atan2((plane.y_pos-self.y_pos),(plane.x_pos-self.x_pos))
                    if (self.heading-0.1) < heading_diff < (self.heading + 0.1): #check wether the other aircraft is flying in front
                        self.v_limit = plane.v
                        self.v_target = self.v_limit
        else:
            self.v_limit = False

    #check if there are new commands given
    def check_newcommands(self,v_max,dt):
        if len(self.op) == 1: #check if there is a command
            for command in self.op:
                if command.status == 1:
                    command.status = command.status + 2
                    self.process_command(command,v_max,dt)
        elif len(self.op) == 0: #whenever there are no commands, determine own speed and heading
            self.heading = self.update_heading()
            self.v_target = False
            self.s_target = False
            self.check_speed(v_max,dt)

    # in the aircraft class
    def process_command(self, command,v_max,dt):
        if command.type == 'heading':
            distance = command.par['distance']
            turn_angle = command.par['turn_angle']
            self.heading_command(distance,turn_angle,v_max,dt)
        elif command.type == 'speed':
            self.speed_command(distance,v_target)

    def heading_command(self,distance,turn_angle,v_max,dt):
        if -0.03*pi < turn_angle < 0.03*pi or  0.97*pi < turn_angle < 1.01*pi or -0.97*pi > turn_angle > -1.01*pi: #if turn angle is smaller then 5 degrees
            self.v_target = v_max
            self.s_target = 0
        elif 0.47*pi < turn_angle < 0.53*pi or -0.47*pi > turn_angle > -0.53*pi: #if turn angle is smaller then 5 degrees
            self.v_target = 0.5144 * int(data[self.type][2]) #set target turn speed
            dcc_dist = 0.5*(self.v_target-self.v)**2/self.dcc + self.v*abs(self.v_target-self.v)/self.dcc #calculate the distance needed to deccelerate
            tot_dist = distance #total distance until next atc
            self.s_target = tot_dist-dcc_dist #the operation starts when the plane's current distance equals the atcdistanve minus the operation distance
        else:
            self.v_target = v_limit
            self.s_target = 0        
            
    def speed_command(self,distance,v_target):
        self.v_target = v_target

    def execute(self,v_max,dt): #execute commands given to this aircraft this timestep
        if self.s_target < v_max*dt:  #if target distance is reached, change speed
            self.check_speed(v_max,dt)
        elif self.s_target == False: #if there is no target distance, change speed
            self.check_speed(v_max,dt)
        else: #if target distance isn't reached, update target distance
            self.s_target = self.s_target - self.v*dt
    
    # update speed
    def check_speed(self,v_max,dt):
        if self.v_target == False:
            if self.v != v_max:
                new_speed = self.v + dt * (float(data[self.type][4])*0.51444) #accelerate to maximum speed
                self.update_speed(new_speed)
        elif self.v < self.v_target:
            new_speed = self.v + dt * (float(data[self.type][4])*0.51444) #accelerate to target speed
            if new_speed > self.v_target:
                self.v = self.v_target
                self.update_speed(self.v_target)
            else:
                self.v = new_speed
                self.update_speed(new_speed)
        elif self.v > self.v_target:
            new_speed = self.v - dt * (float(data[self.type][3])*0.51444) #deccelerate to target speed
            if new_speed < self.v_target:
                self.v = self.v_target
                self.update_speed(self.v_target)
            else:
                self.v = new_speed
                self.update_speed(new_speed)

    # update speed
    def update_speed(self,new_speed):
        self.v = new_speed
                
    # update hading
    def update_heading(self):
        self.heading = atan2((self.y_des-self.y_pos), (self.x_des-self.x_pos))

    # update current atc    
    def update_atc(self,new_atc):
        self.atc[0] = self.atc[1]
        self.atc[1] = new_atc

    # Update positions using velocity and heading (heading is obtained from ATC)
    def update_pos(self,dt):                                           
        if self.heading == False or self.heading == None: #if no heading given, determine heading
            self.update_heading()
        self.x_pos = self.x_pos + self.v * dt * cos(self.heading)
        self.y_pos = self.y_pos + self.v * dt * sin(self.heading)