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
from priodict import priorityDictionary

#import data
from data_import import wp_database
from data_import import wpl_database

class dijkstra_structure:
    def __init__(self):
        self.nodes = []
        self.edges = {}
        self.distances = {}

    def add_node(self, value):
        self.nodes.append(value)

    def add_edge(self, node, v_max): #add edge to the structure, with as value the time it would take to pass the adge at v_max
        links = [elem for elem in wpl_database if int(elem[0]) == node]
        link_dict = {}
        for i in xrange(len(links)):
            value = hypot((wp_database[node][1]-wp_database[links[i][1]][1]),(wp_database[node][2]-wp_database[links[i][1]][2])) / v_max 
            link_dict[links[i][1]] = value
        self.edges[node] = link_dict

def initiate_dijkstra(v_max):
    dijk = dijkstra_structure()
    for i in xrange(len(wp_database)):
        dijk.add_node(wp_database[i][0])
    for node in dijk.nodes:
        dijk.add_edge(node, v_max)
    return dijk

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
    
def update_dijsktra(): #this function should update the structure to the actual situation
    1