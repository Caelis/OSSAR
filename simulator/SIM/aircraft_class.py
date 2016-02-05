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
import time

# Aircraft class is created
class aircraft:

    # Properties of the aircraft 
    def __init__(self,Number,Type,Speed,max_speed,max_acceleration,max_decceleration,X0,Y0,X1,Y1,X2,Y2,heading,ATC_id,ATC_runway):
        self.type = int(Type)
        self.id = Number            # identification number of the aircraft
        self.v = Speed              # current speed of the aircraft
        self.v_target = False       # target speed due to operation
        self.heading = heading      # current heading of the aircraft
        self.heading_target = False # target heading due to commands
        self.s_target = False       # distance at which the aircraft needs to start its operation
        self.acc = max_acceleration # aircrafts maximum acceleration
        self.dcc = max_decceleration# aircrafts maximum deceleration
        self.x_beg = X0             # current link start x_coordinate
        self.y_beg = Y0             # current link start y_coordinate
        self.x_pos = X1             # current x_coordinate
        self.y_pos = Y1             # current y_coordinate
        self.x_des = X2             # current link end x_coordinate
        self.y_des = Y2             # current link end y_coordinate 
        self.atc = [False, ATC_id]  # plane goes from self.atc[0] to self.atc[1]
        self.atc_goal = ATC_runway  # runway goal
        self.op = []                # current operation
        self.radar = []             # stores all planes in radar range
        self.par_avoid = {}         # stores parameters for avoidence manoeuver
        self.par_command = {}       # stores parameters for commanded manoeuver
        self.comfort_deceleration = 2
        self.max_deceleration = 100
        self.comfort_acceleration = 0.8
        self.deceleration = 0
        self.conflict = ''
        self.stop = True           #Becomes True if the aircraft has no goal -> aircraft stops
        self.distance_to_atc = 0    #Distance to the current assigned ATC
        self.isActive = True        # To check if an aircraft is active or not.
        self.handed_off = False     # TO check if aircraft was handed off
        self.ready_for_hand_off = False     # TO check if aircraft is_ready_for_hand_off


    def update(self,separation,v_max,t,dt):
        this_t_stop = 0
        if not self.stop:       # if the plane has a goal_atc, continue to decision making
            self.decision_making(separation,v_max,dt)
        else:                   # if the plane does not have a goal atc, it should stop
            self.deceleration = self.max_deceleration
        #self.update_speed(dt)   # update plane speed
        #self.update_pos(dt)     # update the position of each aircraf decide to accelerate or deceleratet
        if self.v < 0.05:        # each time step calculate the total stopping time
            this_t_stop = dt
        return self.v, this_t_stop

    def check_if_ac_can_stop(self,distance,type):
        if distance >= self.stopping_distance(type):
            return True
        else:
            return False

    def stopping_distance(self,type):
        if type == 'comfort':
            deceleration = self.comfort_deceleration
        elif type == 'emergncy':
            deceleration = self.max_deceleration
        distance = ((self.v)*(self.v))/(2*deceleration)
        return distance



    def process_handoff(self,next_atc,x_beg,y_beg,x_des,y_des):
        self.op = []
        self.par_avoid = {}
        self.par_command = {}
        self.update_atc(next_atc)
        self.v_target = False
        self.s_target = False
        self.heading_target = False
        self.x_beg = x_beg #float(wp_database[self.id][1])
        self.y_beg = y_beg #float(wp_database[self.id][2])
        self.x_des = x_des #float(wp_database[next_atc][1])
        self.y_des = y_des #float(wp_database[next_atc][2])
        self.heading = self.calculate_heading(self.x_des,self.y_des)

    # decision making process to determine heading and speed
    def decision_making(self,separation,v_max,dt):
        self.target_speeds = []
        if self.stop:
            self.deceleration = self.max_deceleration
        else:
            self.deceleration = 0        # always accelerate!
    #        conflict = True
            #  check for collision
            conflict = self.conflict_avoidance(separation)         # perform a collision avoidanace check for all planes within aircraft range
            command = self.check_newcommands(v_max,dt)    # check if new commands were given
            # Here we have 2 boolean variables conflict and command, as well as a dictionary

            needed_deceleration = -1     # assume no deceleration needed
            for speed_command in self.target_speeds:
                if speed_command['s_target'] == 0: #if there is target distance = 0
                    if speed_command['v_target']-self.v == 0:
                        commanded_deceleration = 0
                        print 'check'
                    elif speed_command['v_target']-self.v > 0:
                        commanded_deceleration = -self.comfort_acceleration #TODO acceleration is not needed when waiting in line
                    else:
                        commanded_deceleration = self.max_deceleration
                else:
                    if self.v > 0:
                        commanded_deceleration = (self.v-speed_command['v_target'])/(speed_command['s_target']/self.v) # deceleration based on command
                    else:
                        commanded_deceleration = -(speed_command['v_target']/speed_command['s_target'])
                if commanded_deceleration > needed_deceleration:
                    needed_deceleration = commanded_deceleration
            # print str(self.id) + ': ' + str(self.target_speeds) + ', which is a needed deceleration of: ' + str(needed_deceleration)
            if needed_deceleration == -1:
                self.deceleration = -self.comfort_acceleration
            else:
                if needed_deceleration >= self.comfort_deceleration:
                    if needed_deceleration < self.max_deceleration:
                        # print str(self.id) + ' must decelerate! Current speed = ' + str(self.v)
                        self.deceleration = needed_deceleration
                    else:
        #                print str(self.id) + ' must decelerate at max! Current speed = ' + str(self.v)
                        self.deceleration = self.max_deceleration
                elif needed_deceleration == 0:
                    self.deceleration = 0
                else:
                    self.deceleration = -self.comfort_acceleration


   # checks all planes the radar has detected, which type of conflict would occure when within seperation
    def conflict_avoidance(self,min_separation):
        self.conflict = ''
        conflict = False                                                           # set brake is False (no braking necessary)
        for plane in self.radar:                                                # loop through all planes within radar range
            self_dist = hypot((self.x_pos-self.x_des), (self.y_pos-self.y_des))         # determine own distance to atc
            plane_dist = hypot((plane.x_pos-plane.x_des), (plane.y_pos-plane.y_des))    # determine other plane's distance to/from atc
            if self.atc == plane.atc:                                           # check if planes are on the same link (self.atc1 = plane.atc1 and self.atc2 = plane.atc2)
                conflict = self.conflict_avoidence_link(conflict,plane,min_separation,self_dist,plane_dist)     # execute collision avoidence for same link
                if conflict:
                    self.conflict = 'same'
            elif self.atc[1] == plane.atc[1] and self.atc[0] != plane.atc[0]:   # check if planes have the same goal atc but are not on the same link
                conflict = self.conflict_avoidence_goal(conflict,plane,min_separation,self_dist,plane_dist)     # execute collision avoidence for same goal link but not same current link
                if conflict:
                    self.conflict = 'to same'
            elif self.atc[1] == plane.atc[0]:                                   # check if self.plane
                conflict = self.conflict_avoidence_next(conflict,plane,min_separation,self_dist,plane_dist)     # execute collision avoidence for plane turning onto other planes link
                if conflict:
                    self.conflict = 'on next'
        return conflict

    # determines the necessary avoidence parameters to avoid collision when two planes share the current link
    def conflict_avoidence_link(self,conflict,plane,min_separation,self_dist,plane_dist):
        plane_separation = abs(self_dist - plane_dist)
        if self_dist > plane_dist:
            if plane_separation < (min_separation-25):
                conflict = True                                                         # plane is following and seperation lost --> conflict = True
                v_target = 0                                                      # determine target speed (almost zero to regain seperation if necessary)
                s_target = 0.00001                                                   # determine target distance (almost zero, since direct action)
                self.target_speeds.append({'v_target': v_target, 's_target': s_target})
            elif plane_separation < min_separation:           # check if seperation is lost
                conflict = True                                                         # plane is following and seperation lost --> conflict = True
                v_target = plane.v                                                      # determine target speed (almost zero to regain seperation if necessary)
                s_target = 0.00001                                                   # determine target distance (almost zero, since direct action)
                self.target_speeds.append({'v_target': v_target, 's_target': s_target})
        else:
            conflict = False
        return conflict

    # determines the necessary avoidence parameters to avoid collision when two planes have the same goal link but do not share the current link
    def conflict_avoidence_goal(self,conflict,plane,min_separation,self_dist,plane_dist):
        if self.v > 0:
            try:
                self_arr_time = self_dist/self.v                                            # determine own time to reach goal atc (t = distance/speed)
            except ZeroDivisionError:
                self_arr_time = 1001
            try:                                         # determine other plane's time to reach goal atc
                plane_arr_time = plane_dist/plane.v
            except ZeroDivisionError:
                plane_arr_time = 1000
            self_dist_future = (self_arr_time-plane_arr_time)*self.v                    # distance until next plane when the next plane is at intersection
            if self_dist_future >= 0 and self_dist_future < min_separation:             # check if the other plane will be at the intersection earlier and the distance is smaller then separation min
                max_dcc_dist = 1.5*(self.v)**2/self.comfort_deceleration
                distance_constant_v = self_dist - max_dcc_dist
                v_target = distance_constant_v/plane_arr_time
                s_target = 0
                self.target_speeds.append({'v_target': v_target, 's_target': s_target})
                conflict = True                                                        # plane is second at the intersection and seperation lost --> brake = True

    #            print 'check'
                v_target = plane.v                                                  # speed when other plane is at the intersection should be the same as that plane's speed
                s_target = self_dist + plane_dist - min_separation                                                      # determine target distance (almost zero, since direct action)
                self.target_speeds.append({'v_target': v_target, 's_target': s_target})
        return conflict

    # determines the necessary avoidence parameters to avoid collision this plane turns onto other link with planes within seperation distance
    def conflict_avoidence_next(self,conflict,plane,min_separation,self_dist,plane_dist):
        dcc_dist = 0.5*(plane.v-self.v)**2/self.comfort_deceleration + self.v*abs(plane.v-self.v)/self.comfort_deceleration # calculate the distance needed to deccelerate        
        safe_dist = dcc_dist + min_separation                                       # distance necessary to decelerate and keep seperation   
        plane_dist = hypot((plane.x_pos-self.x_des), (plane.y_pos-self.y_des))    # determine other plane's distance to/from atc
        if plane_dist >= safe_dist:
            conflict = False
        elif plane_dist < safe_dist:                                                  # if the other planes distance to atc is shorter then safe distance:
            if plane_dist < min_separation:
                conflict = True
#                print 'conflict'
                dist_sep = min_separation - plane_dist #distance at which to brake at current link to remain seperation on next link
                s_target = self_dist - (dist_sep + dcc_dist)
                if s_target <= 0:
                    s_target = 0.00001
            else:
                conflict = True
                dist_sep = plane_dist - min_separation
                s_target = (self_dist + plane_dist) - (dcc_dist + min_separation)
                if s_target <= 0:
                    s_target = 0.0001
            v_target = plane.v
            self.target_speeds.append({'v_target': v_target, 's_target': s_target})
        return conflict

    #check if there are new commands given
    def check_newcommands(self,v_max,dt):
        hasCommand = False
        for command in self.op:
            if command.status == 1:                     # if command status = 'send'
                command.status = command.status + 2     # make command status = 'received'
                self.target_speeds.append(self.process_command(command,v_max,dt))  # process the given command
        return hasCommand

    # process given commands
    def process_command(self, command,v_max,dt):
        if command.type == 'heading':                           # if command is a heading command
            distance = command.par['distance']                  # distance at which heading should be changed (given in the command)
            turn_angle = command.par['turn_angle']              # turn angle of the heading change (give in the command, can be zero)
            return self.heading_command(distance,turn_angle,v_max,dt)  # determine operation necessary for executing the command
        elif command.type == 'speed':                           # if command is a speed command
            distance = command.par['distance']                  # distance at which speed should be changed (given in the command)
            v_target = command.par['v_target']                # commanded speed (not yet implemented)
            return self.speed_command(distance,v_target)               # determine operation necessary for executing the command

    #determine operation for heading command
    def heading_command(self,distance,turn_angle,v_max,dt):
        par_command = {}                                        # empty operation parameters
        if -0.03*pi < turn_angle < 0.03*pi or  1.97*pi < turn_angle < 2.03*pi or -1.97*pi > turn_angle > -2.03*pi: #if turn angle is smaller then 5 degrees
            par_command.update({'v_target': v_max})              # operation target speed is maximum speed
            par_command.update({'s_target': distance})                # operation should start immediately (no speed/heading changes)
        elif 0.47*pi < turn_angle < 0.53*pi or -0.47*pi > turn_angle > -0.53*pi or 1.47*pi < turn_angle < 1.53*pi or -1.47*pi > turn_angle > -1.53*pi: #if turn angle is smaller then 5 degrees
            par_command.update({'v_target': 0.5144 * int(data[self.type][2])}) # operation target speed is maximum turning speed
            par_command.update({'s_target': distance})  # operation target distance is the distance until it needs to decelerate
        else:                                                   # if the command is unclear:
            #TODO This happens with a 180 degree turn
            par_command.update({'v_target': v_max})         # operation target speed is speed limit
            par_command.update({'s_target': 0.1})                  # operation should start immediately (no speed/heading changes)
        return par_command

    #determine operation for speed command            
    def speed_command(self,distance,v_target):
        par_command = {}
        par_command.update({'v_target': v_target})
        par_command.update({'s_target': distance})
        return par_command

#    #checks if the collision avoidence or the command is the minimum speed
#    def check_minimumspeed(self,brake,v_max):
#        if self.par_avoid:                                      # check whether there are avoidence parameters
#            self.v_limit = self.par_avoid['v_limit']
#            if brake:
#                self.v_target = self.par_avoid['v_target']  
#                self.s_target = self.par_avoid['s_target']
#            elif self.par_command['s_target'] != 0 or self.v_target == False or self.par_avoid['v_limit'] < self.par_command['v_target'] and self.par_command['s_target'] == 0:
#                self.v_target = self.par_avoid['v_limit']
#                self.s_target = self.par_avoid['s_target']
#            elif self.par_avoid['v_limit'] > self.par_command['v_target'] and self.par_command['s_target'] == 0:
#                self.target = self.par_command['v_target']
#                self.s_target = self.par_command['s_target']
#            else:
#                self.v_target = self.par_command['v_target']
#                self.s_target = self.par_command['s_target']
#        else:
#            if self.par_command:
#                self.v_target = self.par_command['v_target']
#            else:
#                self.v_target = v_max
#                self.s_target = 0.1

    # #execute commands given to this aircraft this timestep
    # def execute(self,v_max,brake,dt):
    #     dcc_dist = 1.5*(v_max)**2/self.dcc          # calculate the maximum distance needed to deccelerate
    #     if not brake and self.s_target > dcc_dist:  # when the plane doesn't have to brake and it still has some distance to taxi until target speed needs to be achieved
    #         if self.v < v_max:
    #             new_speed = self.v + dt * (float(data[self.type][4])*0.51444)   # accelerate to maximum speed
    #             if new_speed > v_max:
    #                 new_speed = v_max
    #             self.update_speed(new_speed)
    #         self.s_target = self.s_target - self.v*dt
    #     elif self.s_target < v_max*dt or self.s_target == False:  #if no target distance/distance is reached, change speed
    #         self.check_speed(v_max,brake,dt)
    #     else:
    #         self.s_target = self.s_target - self.v*dt
  
    # update speed
    # def check_speed(self,v_max,brake,dt):
    #     if self.v_target == False:
    #         if self.v != v_max:
    #             new_speed = self.v + dt * (float(data[self.type][4])*0.51444)   # accelerate to maximum speed
    #             if new_speed > v_max:
    #                 new_speed = v_max
    #             self.update_speed(new_speed)
    #     elif self.v < self.v_target:
    #         new_speed = self.v + dt * (float(data[self.type][4])*0.51444)       # accelerate to target speed
    #         if new_speed > self.v_target:
    #             self.v = self.v_target
    #             self.update_speed(self.v_target)
    #         else:
    #             self.update_speed(new_speed)
    #     elif self.v > self.v_target:
    #         new_speed = self.v - dt * (float(data[self.type][3])*0.51444)       # deccelerate to target speed
    #         if new_speed < self.v_target:
    #             self.v = self.v_target
    #             self.update_speed(self.v_target)
    #         else:
    #             self.update_speed(new_speed)

    # update speed
    def update_speed(self,dt):
        if self.stop:
            self.deceleration = self.max_deceleration
        new_speed = self.v - (dt*self.deceleration)
        if new_speed > 0:
            self.v = new_speed
        else:
            self.v = 0
                
    # update hading
    def update_heading(self):
        # print('Going from [' + str(self.x_pos) + ',' + str(self.y_pos) + '] to [' + str(self.x_des) + ',' + str(self.y_des) + ']')
        self.heading = self.calculate_heading(self.x_des,self.y_des)
        print('Heading: ' + str(self.heading))

    def calculate_heading(self,x_target,y_target):
        heading = atan2((y_target-self.y_pos), (x_target-self.x_pos))
        if self.heading <0:
            self.heading = 2*pi + self.heading
        return heading

    # update current atc    
    def update_atc(self,new_atc):
        self.atc[0] = self.atc[1]
        self.atc[1] = new_atc

    # Update positions using velocity and heading (heading is obtained from ATC)
    def update_pos(self,dt):
        # self.v = self.v - (dt*self.deceleration)
        if self.heading == False or self.heading == None: #if no heading given, determine heading
            self.update_heading()
        self.x_pos = self.x_pos + self.v * dt * cos(self.heading)
        self.y_pos = self.y_pos + self.v * dt * sin(self.heading)