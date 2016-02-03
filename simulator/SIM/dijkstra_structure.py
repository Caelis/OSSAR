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
# import modules
from math import *
import collections
import networkx as nx

# import data
from data_import import wp_database
from data_import import wpl_database


# creates dictionaries of the structure values, structure distances and structure density
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
            distance = hypot((wp_database[source][1] - wp_database[target][1]),
                             (wp_database[source][2] - wp_database[target][2]))
            # compute a value (time) for this edge
            value = distance / v_max
            # add a weighted edge to the graph
            # dijk.add_edge(source,target)
            dijk.add_weighted_edges_from([(source, target, value)])
            dijk[source][target]['distance'] = distance
            dijk[source][target]['density'] = 0
    dijk = nx.DiGraph(dijk)
    return dijk


def  update_dijsktra(ATC_list, graph, seperation, v_max):  # this function updates the graph values for the  current situation
    for key, value in graph.adjacency_iter():
        for inner_key, inner_value in value.items():
            # Get the current density and distance of each link(key, inner_key)
            density = inner_value['density']
            distance = inner_value['distance']
            max_density = distance / seperation

            # Calculate the speeds based on density
            if density > max_density:
                speed = 0
            elif density < 0.5 * max_density:  # TODO make the the 0.5 a variable that is set up in the simulator setup
                speed = v_max
            else:
                if density >= 0:
                    speed = v_max * (1 - (density / max_density))

            # Calculate the graph value based on the speed and distance
            if speed > 0:
                graph[key][inner_key][
                    'weight'] = distance / speed  # TODO this is NOT our optimal soultion!  BALANCING DILEMMA
            else:
                graph[key][inner_key]['weight'] = float('Inf')
    return graph
