# -*- coding: utf-8 -*-
"""
Created on Mon Nov 30 17:04:00 2015

@author: Leo

Package: fleet
Module: runway
module dependence: 

description: Creates a runway class 

Input:
Output:
"""
#import modules
#import itertools

#import data
#from data_import import rw_database

class runway:
    def __init__(self,idnumber,runway_occupance_time):
        self.id = idnumber        
        self.mode = 'take-off'
        self.waiting_list = []          # Array of planes under command
        self.occupance = 0
        self.throughput = 0
        self.default_runway_occupance_time = runway_occupance_time

    def update_occupance(self,dt):
        self.occupance =  self.occupance - dt
        if self.occupance < 0: # checks if there is a occupancy time
            self.occupance = 0
#        print 'runway: ', self.id, ' occupance is: ', self.occupance

    def reset_occupance(self):
        self.occupance = self.default_runway_occupance_time

    def is_occupied(self):
        # print 'occupance is ',self.occupance,' that is why we return '
        if self.occupance > 0:
            # print 'True'
            return True
        else:
            # print 'False'
            return False

def create_runway(idnumber,ATC_list,runway_list,runway_occupance_time):
    runway_nodes = [[18,19],[20,21]]
    for i in xrange(len(runway_nodes)):
        nodes = runway_nodes[i]
        new_runway = runway(idnumber,runway_occupance_time)
        for node in nodes:
            ATC_list[node].runway = new_runway
        runway_list.append(new_runway)
        idnumber = idnumber + 1
    return idnumber, runway_list

def update_runway(runway_list,runway_occupance_time,ATC_list,dt):
    for runway in runway_list:
        runway.update_occupance(dt)