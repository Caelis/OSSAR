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
    def __init__(self,Number,Type,Speed,max_speed,max_acceleration,max_decceleration,X0,Y0,X1,Y1,X2,Y2,heading,ATC_id,ATC_runway,t):
        self.type = int(Type)
        self.id = Number            # identification number of the aircraft
        self.v = Speed              # current speed of the aircraft
        self.v_max = max_speed      # maximum speed of the aircraft
        self.v_max_turn = 10        # maximum turn speed
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
        self.stop = 0           #Becomes True if the aircraft has no goal -> aircraft stops
        self.distance_to_atc = 0    #Distance to the current assigned ATC
        self.is_active = True        # To check if an aircraft is active or not.
        self.handed_off = False     # TO check if aircraft was handed off
        self.ready_for_hand_off = False     # TO check if aircraft is_ready_for_hand_off
        self.last_distance_travelled = 0
        self.target_speeds = []
        self.was_just_handed_off = False
        self.spawntime = t
        self.route = []

    def update(self,separation,v_max,t,dt):
        this_t_stop = 0
        # if not self.stop:       # if the plane has a goal_atc, continue to decision making
        # self.decision_making(separation,v_max,dt)
        self.update_decision_making(separation)
        # else:                   # if the plane does not have a goal atc, it should stop
        # self.deceleration = self.max_deceleration
        #self.update_speed(dt)   # update plane speed
        #self.update_pos(dt)     # update the position of each aircraf decide to accelerate or deceleratet
        if self.v < 0.05:        # each time step calculate the total stopping time
            this_t_stop = dt
        return self.v, this_t_stop

    def update_decision_making(self,separation):
        # # 0. update s_target of exiting commands
        # self.update_commands_distance()
        # print self.deceleration

        # 1. clear speed list
        self.target_speeds = []
        # print self.deceleration

        # 2. Process commands
        self.process_commands()
        # print self.deceleration

        # 3. remove obsolete commands
        self.remove_obsolete_commands()
        # print self.deceleration

        # 4. process comamnds to target speeds
        self.append_commands_to_target_speeds()
        # print self.deceleration
        # print self.target_speeds

        # 5. conflicts to target speeds
        self.conflict_avoidance(separation) # TODO conflict detection
        # print self.deceleration

        # 6. Always try to accelerate to max speed
        self.accelerate_to_v_max()
        # print self.deceleration

        # 7. deceleration decision (based on target speeds)
        self.deceleration_decision(separation)
        # print self.deceleration

    def process_commands(self):
        if len(self.op)>0:
            # print self.op.par
            self.stop = self.stop ^ (self.stop & 64)
            for command in self.op:
                if command.status & 1 and not command.status & 2:                     # if command status = 'send'
                    command.status = command.status | 2     # make command status = 'received'
                elif command.status & 2:
                    command.par['distance'] = command.par['distance'] - self.last_distance_travelled
        else:
            if self.was_just_handed_off:
                self.was_just_handed_off = False
            else:
                self.stop = self.stop | 64


    def remove_obsolete_commands(self):
        for command_1 in self.op:
            for command_2 in self.op:
                if not command_1 == command_2:
                    if command_1.commander == command_2.commander and command_1.type == command_2.type and command_1.send_time <= command_2.send_time:
                        self.op.remove(command_1)
                        break

    def append_commands_to_target_speeds(self):
        for command in self.op:
            to_append = self.process_command(command)
            if to_append:
                self.target_speeds.append(to_append)
            else:
                self.op.remove(command)

    def accelerate_to_v_max(self):
        acceleration = self.v_max-self.v
        if acceleration>self.comfort_acceleration:
            acceleration = self.comfort_acceleration
        self.deceleration = -acceleration

    def deceleration_decision(self,separation):
        if self.stop:
            self.deceleration = self.max_deceleration
            # self.stop_type_decision(separation)
        else:
            temp_deceleration = 0

            # loop through target speeds and find highest necessary deceleration
            for target_speed in self.target_speeds:
                this_deceleration = 0-self.calc_acceleration(target_speed['v_target'],target_speed['s_target'])# ()(self.v-target_speed['v_target'])**2)/(2*target_speed['s_target'])
                if this_deceleration > temp_deceleration:
                    temp_deceleration = this_deceleration
            # check if the aircraft should decelerate
            if temp_deceleration >= self.comfort_deceleration:
                if temp_deceleration <= self.max_deceleration:
                    self.deceleration = temp_deceleration
                else:
                    self.deceleration = self.max_deceleration

    def stop_type_decision(self,separation):
        # After handoff threshold
        if self.stop & 1:
            self.is_active = False
        # after pre-handoff
        if self.stop & 2:
            self.deceleration = self.max_deceleration
        # depreciated
        if self.stop & 4:
            self.deceleration = self.max_deceleration
        # no successful path planned
        if self.stop & 8:
            self.deceleration = self.calc_acceleration(0, self.distance_to_atc-separation)
        # runway busy
        if self.stop & 16:
            self.deceleration = self.calc_acceleration(0, self.distance_to_atc-separation)
        # depreciated
        if self.stop & 32:
            self.deceleration = self.max_deceleration
        # no Operation path planned
        if self.stop & 64:
            self.deceleration = self.calc_acceleration(0, self.distance_to_atc-separation)
        # no path at pre-handoff
        if self.stop & 128:
            self.deceleration = self.max_deceleration
        # unsolvable conflict
        if self.stop & 256:
            self.is_active = False
        # Gate busy
        if self.stop & 512:
            self.is_active = False

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

    def calc_acceleration(self,v_goal,distance):
        if not distance == 0:
            # acceleration = (1/2*(v_delta**2)+self.v*v_delta)/distance
            # acceleration = (1/2*(v_goal**2-2*v_goal*self.v+self.v**2)+self.v*v_delta)/distance
            acceleration = (v_goal*v_goal-self.v*self.v)/(2*distance)
        else:
            acceleration = 0-self.max_deceleration
        # TODO print 'v_g: ',v_goal,' v: ',self.v,' d: ',distance,' a:',acceleration
        return acceleration

    def process_handoff(self,next_atc,x_beg,y_beg,x_des,y_des):
        # print len(self.route)>0,self.route[0],self.atc[0],self.route[1],next_atc
        # if self.route[0] == False:
        #     self.route = self.route[1:]
        if len(self.route)>0 and int(self.route[0][0]) == self.atc[0] and int(self.route[1][0]) == next_atc:
        # elif len(self.route)>0 and int(self.route[0][0]) == self.atc[0] and int(self.route[1][0]) == next_atc:
            self.route = self.route[1:]
        self.target_speeds = []
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
        self.was_just_handed_off =True


   # checks all planes the radar has detected, which type of conflict would occure when within seperation
    def conflict_avoidance(self,min_separation):
        self.conflict = ''
        conflict = False                                                           # set brake is False (no braking necessary)
        for plane in self.radar:                                                # loop through all planes within radar range
            # self_dist = hypot((self.x_pos-self.x_des), (self.y_pos-self.y_des))         # determine own distance to atc
            self_dist = self.distance_to_atc
            # plane_dist = hypot((plane.x_pos-plane.x_des), (plane.y_pos-plane.y_des))    # determine other plane's distance to/from atc
            plane_dist = plane.distance_to_atc
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

    # #check if there are new commands given
    # def check_newcommands(self,v_max,dt):
    #     # print len(self.op)
    #     for command in self.op:
    #         if command.status == 1:                     # if command status = 'send'
    #             command.status = command.status + 2     # make command status = 'received'
    #             to_append = self.process_command(command,v_max,dt)
    #             self.target_speeds.append(to_append)  # process the given command

    def cleanup_aircraft_accelerationtarget_speeds(self):
        current_array = []
        for command_1 in self.target_speeds:
            thisCommand = False
            for command_2 in self.target_speeds:
                pass

    # process given commands
    def process_command(self, command):
        if command.type == 'heading':                           # if command is a heading command
            # print 'T d: ',command.par['distance'],' h:',command.par['turn_angle']
            distance = command.par['distance']                  # distance at which heading should be changed (given in the command)
            turn_angle = command.par['turn_angle']              # turn angle of the heading change (give in the command, can be zero)
            return self.heading_command(distance,turn_angle)  # determine operation necessary for executing the command
        elif command.type == 'speed':
            # print 'S d: ',command.par['distance'],' v:',command.par['v_target']                           # if command is a speed command
            distance = command.par['distance']                  # distance at which speed should be changed (given in the command)
            v_target = command.par['v_target']                # commanded speed (not yet implemented)
            return self.speed_command(distance,v_target)  # determine operation necessary for executing the command
        elif command.type == 'route':
            # Only accept routes to the destination as new route!
            if self.atc_goal == command.par['route'][-1] or str(self.atc_goal) == command.par['route'][-1]:
                self.route = command.par['route']
            return False


    #determine operation for heading command
    def heading_command(self,distance,turn_angle):
        par_command = {}                                        # empty operation parameters
        if -0.03*pi < turn_angle < 0.03*pi or  1.97*pi < turn_angle < 2.03*pi or -1.97*pi > turn_angle > -2.03*pi: #if turn angle is smaller then 5 degrees
            par_command.update({'v_target': self.v_max})              # operation target speed is maximum speed
            par_command.update({'s_target': distance})                # operation should start immediately (no speed/heading changes)
        elif 0.47*pi < turn_angle < 0.53*pi or -0.47*pi > turn_angle > -0.53*pi or 1.47*pi < turn_angle < 1.53*pi or -1.47*pi > turn_angle > -1.53*pi: #if turn angle is smaller then 5 degrees
            par_command.update({'v_target': 0.5144 * int(data[self.type][2])}) # operation target speed is maximum turning speed
            par_command.update({'s_target': distance})  # operation target distance is the distance until it needs to decelerate
            # print par_command
        else:                                                   # if the command is unclear:
            #TODO This happens with a 180 degree turn
            par_command.update({'v_target': self.v_max})         # operation target speed is speed limit
            par_command.update({'s_target': 0.1})                  # operation should start immediately (no speed/heading changes)
        return par_command

    #determine operation for speed command            
    def speed_command(self,distance,v_target):
        par_command = {}
        par_command.update({'v_target': v_target})
        par_command.update({'s_target': distance})
        return par_command

    # update speed
#    def update_speed(self,Aircraft_acceleration,dt):
#        current_speed = self.v
#        new_speed = self.v - (dt*self.deceleration)
#        #tracks how many aircraft during the simulation accelerate from full stop
#        if current_speed == 0 and new_speed > 0:
#            Aircraft_acceleration = Aircraft_acceleration + 1
#        if new_speed > 0:
#            if new_speed > self.v_max:
#                self.v = self.v_max
#            else:
#                self.v = new_speed
#        else:
#            self.v = 0
#        return Aircraft_acceleration
                
    # update hading
    def update_heading(self):
        if not self.stop:
            # print('Going from [' + str(self.x_pos) + ',' + str(self.y_pos) + '] to [' + str(self.x_des) + ',' + str(self.y_des) + ']')
            self.heading = self.calculate_heading(self.x_des,self.y_des)
            # print('Heading: ' + str(self.heading))

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
        self.last_distance_travelled = self.v * dt
        for speed_command in self.target_speeds:
            speed_command['s_target'] = speed_command['s_target']-self.last_distance_travelled

    def update_pos_speed(self,aircraft_accelerating,dt):
        # self.v = self.v - (dt*self.deceleration)
        if self.heading == False or self.heading == None: #if no heading given, determine heading
            self.update_heading()

        # for distance calculation
        x_pos_old = self.x_pos
        y_pos_old = self.y_pos
        
        # for tracking of variables
        v_old = self.v
        
        # update position
        delta_s = -0.5*self.deceleration*dt*dt+self.v*dt
        if delta_s<0:
            delta_s = 0
        self.x_pos = self.x_pos + delta_s * cos(self.heading)
        self.y_pos = self.y_pos + delta_s * sin(self.heading)

        # update speed
        delta_v = -self.deceleration*dt
        self.v = self.v + delta_v
        if self.v <0 :
            self.v = 0
        
        # self.last_distance_travelled = hypot(self.x_pos-x_pos_old,self.y_pos-y_pos_old)
        self.last_distance_travelled = delta_s
        
        # tracks how many aircraft during the simulation accelerate from full stop
        if v_old == 0 and self.v > 0:
            aircraft_accelerating = aircraft_accelerating + 1
        return aircraft_accelerating