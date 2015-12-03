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
from priodict import priorityDictionary

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
        links = [elem for elem in wpl_database if int(elem[0]) == node]
        link_dict = {}
        dist_dict = {}
        dens_dict = {}
        for i in xrange(len(links)):
            distance = hypot((wp_database[node][1]-wp_database[links[i][1]][1]),(wp_database[node][2]-wp_database[links[i][1]][2]))
            value =  distance / v_max 
            link_dict[links[i][1]] = value
            dist_dict[links[i][1]] = distance
            dens_dict[links[i][1]] = 0
        self.edges[node] = link_dict
        self.distances[node] = dist_dict
        self.density[node] = dens_dict

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

def dijkstra(G,start,end):
    D = {}	# dictionary of final distances
    P = {}	# dictionary of predecessors
    Q = priorityDictionary()	# estimated distances of non-final vertices

    # initialize Q and P
    for vertex in G:
        Q[vertex] = float("inf")
        P[vertex] = None
    Q[start] = 0

    for v in Q: #loop through all waypoints
        D[v] = Q[v]
        if v == end:    
            break
        for w in G[v]: #loop through all links for waypoint
            vwLength = D[v] + G[v][w]
            if w in D:
                if vwLength < D[w]:
                    raise ValueError, "Dijkstra: found better path to already-final vertex"
            elif w not in Q or vwLength < Q[w]:
                Q[w] = vwLength
                P[w] = v
    return (D,P)
			
def shortestpath(G,start,end):
    D,P = dijkstra(G,start,end)
    Path = []
    while 1:
        Path.append(end)
        if end == start: 
            break
        end = P[end]
    Path.reverse()
    return Path

def update_dijsktra(ATC_list,structure,struc_dens0,struc_dist,seperation,v_max): #this function updates the structure to the actual situation
    struc_dens = deepcopy(struc_dens0)
    for atc in ATC_list: #create density dictionary
        for plane in atc.locp:
            if ATC_list[plane.atc[1]].type != 4:
                struc_dens[plane.atc[0]][plane.atc[1]] = struc_dens[plane.atc[0]][plane.atc[1]] + 1 #adds one to the link the plane is on
    for key, value in struc_dens.iteritems(): #change to speed dictionary using Greenshield's model     
        for inner_key, inner_value in value.iteritems():
            density = struc_dens[key][inner_key]
            max_density = struc_dist[key][inner_key]/seperation
            if density > max_density:
                struc_dens[key][inner_key] = 0
            else:
                struc_dens[key][inner_key] = v_max * (1 - (density/max_density) ) #Greenshield's model for flow speed
    for key, value in structure.iteritems(): #create actuel values for Dijkstra's algorithm
        for inner_key,inner_value in value.iteritems():
            speed = struc_dens[key][inner_key]
            distance = struc_dist[key][inner_key]
            structure[key][inner_key] = distance/speed
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