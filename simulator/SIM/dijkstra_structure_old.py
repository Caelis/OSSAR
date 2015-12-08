# -*- coding: utf-8 -*-
"""
Created on Thu Nov 12 09:03:20 2015

@author: Leo

Package: atc
Module: dijkstra
module dependence: atc_class

description: implementation of the Dijkstra algorithm

Input:
Output:
"""
#import modules
from math import *
from copy import deepcopy
import collections
# from priodict import priorityDictionary
from dijkstra import *

#import data
from data_import import wp_database
from data_import import wpl_database

class dijkstra_structure:
    def __init__(self):
        self.nodes = []
        self.edges = {}
        self.distances = {} #dictionary of the distances of each edge
        self.density = {} #dictionary of the density of each edge

    def add_node(self, value):
        self.nodes.append(value)

    #create all edges and add a value depending on speed
    def add_edge(self, node, v_max): #add edge to the structure, with as value the time it would take to pass the adge at v_max
        # get the links that are connected to this node
        links = [elem for elem in wpl_database if int(elem[0]) == node]
        # This if clauses are to make sure to not overwrite the "return" edges
        # if self.edges.has_key(node):
        #     link_dict = self.edges[node]
        # else:
        #     link_dict = {}
        # if self.distances.has_key(node):
        #     dist_dict = self.distances[node]
        # else:
        #     dist_dict = {}
        # if self.density.has_key(node):
        #     dens_dict = self.density[node]
        # else:
        #     dens_dict = {}
        # loop through all links of this node
        for i in xrange(len(links)):
            # calulate the distanc of this node
            distance = hypot((wp_database[node][1]-wp_database[links[i][1]][1]),(wp_database[node][2]-wp_database[links[i][1]][2]))
            # compute a value for this node
            value =  distance / v_max
            # add the edges in "positive" direction
            self.edges = update(self.edges,{node: {links[i][1]: value}})
            self.distances = update(self.distances,{node: {links[i][1]: distance}})
            self.density = update(self.density,{node:{links[i][1]: 0}})
            # add the edges in "negative" direction
            self.edges = update(self.edges,{links[i][1]: {node: value}})
            self.distances = update(self.distances,{links[i][1]: {node: distance}})
            self.density = update(self.density,{links[i][1]:{node: 0}})
        print self.edges

#creates dictionaries of the structure values, structure distances and structure density
def initiate_dijkstra(v_max):
    dijk = dijkstra_structure()
    for i in xrange(len(wp_database)):
        dijk.add_node(wp_database[i][0])
    for node in dijk.nodes:
        dijk.add_edge(node, v_max)
    structure = dijk.edges #create a dictionary of the starting structure for the Dijkstra algorithm
    struc_dist = dijk.distances
    struc_dens = dijk.density
    return structure, struc_dist, struc_dens

def update_dijsktra(ATC_list,structure,struc_dens0,struc_dist,seperation,v_max): #this function updates the structure to the actual situation
    struc_dens = deepcopy(struc_dens0)
    for atc in ATC_list: #create density dictionary
        for plane in atc.locp:
            if ATC_list[plane.atc[1]].type != 4:
                struc_dens[plane.atc[0]][plane.atc[1]] = struc_dens[plane.atc[0]][plane.atc[1]] + 1 #adds one to the link the plane is on
                struc_dens[plane.atc[1]][plane.atc[0]] = struc_dens[plane.atc[1]][plane.atc[0]] - 1 #removes one to the oppsoite link the plane is on

    for key, value in struc_dens.iteritems(): #change to speed dictionary using Greenshield's model
        for inner_key, inner_value in value.iteritems():
            # get the density from previous step
            density = struc_dens[key][inner_key]
            distance = struc_dist[key][inner_key]
            # Compute the max_density based on distance and separation
            max_density = distance/seperation
            # if density too high, set "spped" to 0
            if density > max_density:
                # struc_dens[key][inner_key] = 0
                speed = 0
            # Otherwise calculate speed based on density
            else:
                if density >= 0:
                    # struc_dens[key][inner_key] = v_max * (1 - (density/max_density) ) #Greenshield's model for flow speed
                    speed = v_max * (1 - (density/max_density) )
                else:
                    # struc_dens[key][inner_key] = -v_max * (1 - (abs(density)/max_density) ) #Greenshield's model for flow speed
                    speed = 0
            # Save computed time
            if speed > 0:
                structure[key][inner_key] = distance/speed
            else:
                if structure[key].has_key(inner_key):
                    structure[key].pop(inner_key)
    # for key, value in structure.iteritems(): #create actuel values for Dijkstra's algorithm
    #     for inner_key,inner_value in value.iteritems():
    #         speed = struc_dens[key][inner_key]
    #         distance = struc_dist[key][inner_key]
    #         structure[key][inner_key] = distance/speed
    return structure
    
#def nested_dict_Greenshield(nested,struc_dens,struc_dist,seperation,v_max):  #change to speed dictionary using Greenshield's model
#    for key, value in nested.iteritems():
#        if isinstance(value, collections.Mapping):
#            for inner_key, inner_value in nested_dict_Greenshield(value,struc_dens,struc_dist,seperation,v_max):
#                density = struc_dens[key][inner_key]
#                max_density = struc_dist[key][inner_key]/seperation
#                if density > max_dens:
#                    struc_dens[key][inner_key] = 0
#                else:
#                    struc_dens[key][inner_key] = v_max * (1 - (density/max_density) ) #Greenshield's model for flow speed
#    return struc_dens
#
#def nested_dict_structure(nested,structure,struc_dens,struc_dist):  #create actuel values for Dijkstra's algorithm
#    for key, value in nested.iteritems():
#        if isinstance(value, collections.Mapping):
#            for inner_key, inner_value in nested_dict_structure(value,structure,struc_dens,struc_dist):
#                speed = struc_dens[key][inner_key]
#                distance = struc_dist[key][inner_key]
#                structure[key][inner_key] = distance/speed
#    return structure

def update(d, u):
    # update nested dictionary "d" with vaule pair "u" (http://stackoverflow.com/questions/3232943/update-value-of-a-nested-dictionary-of-varying-depth)
    for k, v in u.iteritems():
        if isinstance(v, collections.Mapping):
            r = update(d.get(k, {}), v)
            d[k] = r
        else:
            d[k] = u[k]
    return d