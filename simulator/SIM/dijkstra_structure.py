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
import collections

import networkx as nx


#import data
from data_import import wp_database
from data_import import wpl_database

#creates dictionaries of the structure values, structure distances and structure density
def initiate_dijkstra(v_max):
    # create an new graph "dijk"
    dijk = nx.Graph()
    # loop through nodes in "wp_database"
    for i in xrange(len(wp_database)):
        source = wp_database[i][0]
        # Add nodes from the "wp_database"
        dijk.add_node(source)
        # get node links from "wpl_databse"
        links = [elem for elem in wpl_database if int(elem[0]) == source]
        # loop thorough all links
        for j in xrange(len(links)):
            target = links[j][1]
            # calulate the distanc of the edge from source to target
            distance = hypot((wp_database[source][1]-wp_database[target][1]),(wp_database[source][2]-wp_database[target][2]))
            # compute a value (time) for this edge
            value =  distance / v_max
            # add a weighted edge to the graph
            # dijk.add_edge(source,target)
            dijk.add_weighted_edges_from([(source,target,value)])
            dijk[source][target]['distance']=distance
            dijk[source][target]['density']=0
    dijk = nx.DiGraph(dijk)
    return dijk

def update_dijsktra(ATC_list,graph,seperation,v_max): #this function updates the structure to the current situation
    for atc in ATC_list: #create density dictionary
        for plane in atc.locp:
            if ATC_list[plane.atc[1]].type != 4:
                graph[plane.atc[0]][plane.atc[1]]['density'] = graph[plane.atc[0]][plane.atc[1]]['density'] + 1
                graph[plane.atc[1]][plane.atc[0]]['density'] = graph[plane.atc[1]][plane.atc[0]]['density'] - 1
    for key,value in graph.adjacency_iter():
        for inner_key,inner_value in value.items():
            density = inner_value['density']
            distance = inner_value['distance']
            max_density = distance/seperation
            if density > max_density:
                speed = 0
            else:
                if density >= 0:
                    speed = v_max * (1 - (density/max_density) )
                else:
                    speed = -1
            if speed > 0:
                graph[key][inner_key]['weight'] = distance/speed # TODO this is NOT our optimal soultion!  BALANCING DILEMMA
            elif speed == 0:
                graph[key][inner_key]['weight'] = float('Inf')
            else:
                graph.remove_edge(key,inner_key)
                # graph[key][inner_key]['weight'] = float('Inf')
    # print 'to 5 = '+ str(graph[6][5]['weight'])
    # print 'to 7 = '+ str(graph[6][7]['weight'])
    # print 'to 23 = '+ str(graph[6][23]['weight'])

    return graph

def update(d, u):
    # update nested dictionary "d" with vaule pair "u" (http://stackoverflow.com/questions/3232943/update-value-of-a-nested-dictionary-of-varying-depth)
    for k, v in u.iteritems():
        if isinstance(v, collections.Mapping):
            r = update(d.get(k, {}), v)
            d[k] = r
        else:
            d[k] = u[k]
    return d