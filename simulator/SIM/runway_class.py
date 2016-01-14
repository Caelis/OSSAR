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
    def __init__(self,idnumber,nodes):
        self.id = idnumber        
        self.mode = 'take-off'
        self.waiting_list = []          # Array of planes under command
        self.nodes = nodes
        self.occupance = False
        self.throughput = 0

    def add_nodes(self,nodes):
        self.nodes.append(nodes)

    def update_occupance(self,dt):
        if self.occupance <= 0: # checks if there is a occupancy time
            self.occupance = False
        else:
            self.occupance =  self.occupance - dt  # if True, count down
#        print 'runway: ', self.id, ' occupance is: ', self.occupance
    
    def take_off(self,plane,runway_occupance_time,ATC_list):
        if self.waiting_list: #If there is a waiting list
            if not self.occupance:  # If the runway is not occupied
                plane = self.waiting_list[0]
                self.occupance = runway_occupance_time       # set occupancy time
                ATC_list[plane.atc[1]].remove_plane(plane)  # remove plane from ATC
                self.waiting_list.remove(plane)         # remove plane from waiting_list
            else:
                plane.stop = True

def create_runway(idnumber,ATC_list,runway_list):
    runway_nodes = [[18,19],[20,21]]    
    for i in xrange(len(runway_nodes)):
        nodes = runway_nodes[i]
        for node in nodes:
            ATC_list[node].par.append({'runway_id': idnumber})
        runway_list.append(runway(idnumber,nodes))
        idnumber = idnumber + 1
    return idnumber, runway_list

def update_runway(runway_list,runway_occupance_time,ATC_list,dt):
    for runway in runway_list:
        runway.update_occupance(dt)
#        runway.take_off(runway_occupance_time,ATC_list)
#        print 'runway ', runway.id, ' with waiting list: ', runway.waiting_list