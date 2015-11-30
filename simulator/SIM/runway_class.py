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
import itertools

#import data
from data_import import rw_database

class runway:
    def __init__(self,runway_throughput):
        self.mode = 'take-off'
        self.nodes = nodes
        self.occupance = False
        self.throughput = runway_throughput
            