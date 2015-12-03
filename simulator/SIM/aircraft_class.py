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
        self.v_target = False       #target speed due to operation
        self.heading = heading      #current heading of the aircraft
        self.heading_target = False #target heading due to commands
        self.s_target = False       #distance at which the aircraft needs to start its operation
        self.acc = max_acceleration #aircrafts maximum acceleration
        self.dcc = max_decceleration#aircrafts maximum deceleration
        self.x_beg = X0             #begin x_coordinate
        self.y_beg = Y0             #begin y_coordinate
        self.x_pos = X1             #current x_coordinate
        self.y_pos = Y1             #current y_coordinate
        self.x_des = X2             #current x_coordinate of destination
        self.y_des = Y2             #current y_coordinate of destination 
        self.atc = [False, ATC_id]  #plane goes from self.atc[0] to self.atc[1]
        self.atc_goal = ATC_runway  #runway goal
        self.op = []                #current operation
        self.par_avoid = {}         #stores parameters for avoidence manoeuver
        self.par_command = {}       #stores parameters for commanded manoeuver

    # decision making process to determine heading and speed
    def decision_making(self,ATC_list,separation,v_max,dt): 
        brake = self.collision_avoidence(ATC_list,separation,v_max) # check if there are any aircraft within seperation minimum (aircraft should brake if True)
        self.check_newcommands(v_max,dt)                # check if new commands were given
        self.check_minimumspeed(brake,v_max)            # check if the operation speed or avoidence speed is the lowest and set as targetspeed
        if len(self.op) != 0 or len(self.par_avoid) != 0:# if there are commands or if there is no avoidence necessary:
            self.execute(v_max,dt)                      # execute given commands
        
    #check if there are any aircraft at or within seperation minimum and if so, change the speed limit
    def collision_avoidence(self,ATC_list,separation,v_max):
        min_separation = separation                     # minimal seperation [m] for taxiing aircraft
        brake = False                                   # set brake is False (no braking necessary)
        self.par_avoid = {}                             # if avoidence is nessecary, store the parameters in a dictionary
        if len(ATC_list[self.atc[1]].locp) > 1:
            for plane in ATC_list[self.atc[1]].locp:
                plane_separation = hypot((self.x_pos-plane.x_pos), (self.y_pos-plane.y_pos))
                if plane_separation < min_separation and plane.id != self.id:       # check if seperation is lost
                    heading_diff = atan2((plane.y_pos-self.y_pos),(plane.x_pos-self.x_pos))
                    if (self.heading-0.1) < heading_diff < (self.heading + 0.1):    # check wether the other aircraft is flying in front
                        brake = True
                        self.par_avoid['v_limit'] = plane.v
                        self.par_avoid['v_target'] = 0.1
                        self.par_avoid['s_target'] = 0.1
        else:
            self.v_limit = False
        return brake

    #check if there are new commands given
    def check_newcommands(self,v_max,dt):
        if len(self.op) == 1:           # check if there is a command
            for command in self.op:     # check for each command
                if command.status == 1: # if command status = 'send'
                    command.status = command.status + 2     # make command status = 'received'
                    self.process_command(command,v_max,dt)  # process the given command
        elif len(self.op) == 0:         # whenever there are no commands, determine own speed and heading
            self.heading = self.update_heading()    # update heading according to target
            self.v_target = False       # Empty target speed
            self.s_target = False       # Empty target distance
            self.check_speed(v_max,dt)  # update speed

    # process given commands
    def process_command(self, command,v_max,dt):
        if command.type == 'heading':               # if command is a heading command
            distance = command.par['distance']      # distance at which heading should be changed (given in the command)
            turn_angle = command.par['turn_angle']  # turn angle of the heading change (give in the command, can be zero)
            self.heading_command(distance,turn_angle,v_max,dt)  # determine operation necessary for executing the command 
        elif command.type == 'speed':               # if command is a speed command (not yet implemented)
            self.speed_command(distance,v_target)   # determine operation necessary for executing the command 

    #determine operation for heading command
    def heading_command(self,distance,turn_angle,v_max,dt):
        par_command = {}                                    # empty operation parameters
        if -0.03*pi < turn_angle < 0.03*pi or  0.97*pi < turn_angle < 1.01*pi or -0.97*pi > turn_angle > -1.01*pi: #if turn angle is smaller then 5 degrees
            self.par_command['v_target'] = v_max            # operation target speed is maximum speed
            self.par_command['s_target'] = 0.1              # operation should start immediately (no speed/heading changes)
            self.v_target = self.par_command['v_target']    # set aircraft target speed to operation target speed
            self.s_target = self.par_command['s_target']    # set aircraft target distance to operation target distance
        elif 0.47*pi < turn_angle < 0.53*pi or -0.47*pi > turn_angle > -0.53*pi or 1.47*pi < turn_angle < 1.53*pi or -1.47*pi > turn_angle > -1.53*pi: #if turn angle is smaller then 5 degrees
            self.par_command['v_target'] = 0.5144 * int(data[self.type][2]) # operation target speed is maximum turning speed
            self.v_target = self.par_command['v_target']    # set aircraft target speed to operation target speed
            dcc_dist = 0.5*(self.v_target-self.v)**2/self.dcc + self.v*abs(self.v_target-self.v)/self.dcc #calculate the distance needed to deccelerate
            tot_dist = distance                             # distance at which the operation should be completed
            self.par_command['s_target'] = tot_dist - dcc_dist# operation target distance is the distance until it needs to decelerate
            self.s_target = self.par_command['s_target']    # set aircraft target distance to operation target distance
        else:                                               # if the command is unclear:
            self.par_command['v_target'] = self.v_limit     # operation target speed is speed limit
            self.par_command['s_target'] = 0.1              # operation should start immediately (no speed/heading changes)
            self.v_target = self.par_command['v_target']    # set aircraft target speed to operation target speed
            self.s_target = self.par_command['s_target']    # set aircraft target distance to operation target distance

    #determine operation for speed command            
    def speed_command(self,distance,v_target):
        self.v_target = v_target

    #checks if the collision avoidence or the command is the minimum speed
    def check_minimumspeed(self,brake,v_max):
        if self.par_avoid: #check whether there are avoidence parameters
            self.v_limit = self.par_avoid['v_limit']
            if brake:
                self.v_target = self.par_avoid['v_target']  
                self.s_target = 0.1
            elif self.par_command['s_target'] != 0 or self.v_target == False or self.par_avoid['v_limit'] < self.par_command['v_target'] and self.par_command['s_target'] == 0:
                self.v.target = self.par_avoid['v_limit']
                self.s_target = self.par_avoid['s_target']                 
            elif self.par_avoid['v_limit'] > self.par_command['v_target'] and self.par_command['s_target'] == 0:
                self.target = self.par_command['v_target']
                self.s_target = 0.1
            else:
                self.v.target = self.par_command['v_target']
                self.s_target = self.par_command['s_target']
        else:
            if self.par_command:
                self.v_target = self.par_command['v_target']
            else:
                self.v_target = v_max
                self.s_target = 0.1

    #execute commands given to this aircraft this timestep
    def execute(self,v_max,dt): 
        if self.s_target < v_max*dt or self.s_target == False:  #if no target distance/distance is reached, change speed
            self.check_speed(v_max,dt)
        else: 
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