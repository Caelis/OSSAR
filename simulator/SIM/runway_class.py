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
    def __init__(self,idnumber,runway_throughput):
        self.id = idnumber        
        self.mode = 'take-off'
        self.nodes = []
        self.occupance = False
        self.throughput = runway_throughput

    def add_nodes(nodes):
        self.nodes.append(nodes)
    
    def determine_occupance(t):
        a

def create_runway(idnumber,runway_list,runway_throughput):
#    runway_inst = runway(runway_throughput)
#    runway_list.append(runway_inst)
    runway_nodes = [[18,19],[20,21]]    
    for elem in runway_nodes:
        runway_inst = runway(elem,runway_throughput)
        runway_list.append(runway_inst)
        idnumber = idnumber + 1
    return idnumber, runway_list