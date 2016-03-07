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
            dijk[source][target]['speed'] = v_max
    dijk = nx.DiGraph(dijk)
    return dijk


def calc_heading(x_1,y_1,x_2,y_2):
    heading = atan2((y_2-y_1), (x_2-x_1))
    return heading

def add_dummy_edges(graph,atc_list, default_values):
    # TODO add original nodes as well, and connect to all possible connections
    v_turn = default_values['v_turn']
    acc_standard = default_values['acc_standard']
    dec_standard = default_values['dec_standard']

    largeGraph = nx.DiGraph()
    pos = {}
    turn_distance_added = 0.1

    for key, value in graph.adjacency_iter():
        # Add real node to graph
        if not largeGraph.has_node(str(key)):
            largeGraph.add_node(str(key))
            nx.set_node_attributes(largeGraph,'main',{str(key): key})
            pos[str(key)]=(wp_database[key][1],wp_database[key][2])
        # print key
        for inner_key, inner_value in value.items():
            graph[key][inner_key]['linked_edges'] = []
            heading_1 = calc_heading(wp_database[key][1],wp_database[key][2],wp_database[inner_key][1],wp_database[inner_key][2])
            # print '-',inner_key
            # check if this is a start/end node
            if len(value.items()) == 1:
                node_label_1 = str(key)
            else:
                node_label_1 = str(key) + '_' + str(inner_key)

            # Add virtual node to new graph check if the node is in the graph already
            if not largeGraph.has_node(node_label_1):
                largeGraph.add_node(node_label_1)
                nx.set_node_attributes(largeGraph,'main',{node_label_1: key})
                pos[node_label_1]=(wp_database[key][1],wp_database[key][2])

            # loop through 2nd degree connections
            for conn in graph[inner_key]:
                heading_2 = calc_heading(wp_database[inner_key][1],wp_database[inner_key][2],wp_database[conn][1],wp_database[conn][2])


                # print '--',conn
                # print conn,type(conn)
                # print key,type(key)
                if not (conn == key and len(graph[inner_key]) > 1):
                    # print 'Adding',key,inner_key,conn
                    if len(graph[inner_key]) > 1:
                        node_label_2 = str(inner_key) + '_' + str(conn)
                    else:
                        node_label_2 = str(inner_key)

                    # add node if necessary
                    if not largeGraph.has_node(node_label_2):
                        largeGraph.add_node(node_label_2)
                        nx.set_node_attributes(largeGraph,'main',{node_label_2: inner_key})
                        pos[node_label_2]=(wp_database[inner_key][1],wp_database[inner_key][2])

                    # don't add edges towards gates, or from runways
                    # TODO this is a temorary fix for the issue that paths were assigned through gates/through runways. This won't work for mixed arr/dep traffic
                    if not (atc_list[inner_key].type == 1 or atc_list[key].type == 4):
                        # add virtual edge if necessary
                        if not largeGraph.has_edge(node_label_1,node_label_2):
                            largeGraph.add_edge(node_label_1,node_label_2)
                            # add "turn" value to each edge to see if it was a turn
                            if not heading_1 == heading_2:
                                largeGraph[node_label_1][node_label_2]['turn'] = True
                                turn_distance = turn_distance_added
                            else:
                                largeGraph[node_label_1][node_label_2]['turn'] = False
                                turn_distance = 0

                            largeGraph[node_label_1][node_label_2]['original_edge'] = [key,inner_key]
                            graph[key][inner_key]['linked_edges'].append([node_label_1,node_label_2])

                            # get values of original link, compute for this link and add to this link
                            distance = graph[key][inner_key]['distance']
                            speed = graph[key][inner_key]['speed']

                            time_for_link = time_with_turn_penalty(distance,speed,v_turn,dec_standard,acc_standard,largeGraph[node_label_1][node_label_2]['turn'])

                            largeGraph[node_label_1][node_label_2]['distance'] = distance
                            largeGraph[node_label_1][node_label_2]['density'] = graph[key][inner_key]['density']
                            largeGraph[node_label_1][node_label_2]['weight'] = time_for_link

                            # add real edge if necessary
                            if not largeGraph.has_edge(key,node_label_2):
                                thisKey = str(key)
                                largeGraph.add_edge(thisKey,node_label_2)
                                largeGraph[thisKey][node_label_2]['distance'] = distance
                                largeGraph[thisKey][node_label_2]['turn'] = largeGraph[node_label_1][node_label_2]['turn']
                                largeGraph[thisKey][node_label_2]['density'] = graph[key][inner_key]['density']
                                largeGraph[thisKey][node_label_2]['weight'] = time_for_link
                                largeGraph[thisKey][node_label_2]['original_edge'] = [key,inner_key]
                                graph[key][inner_key]['linked_edges'].append([thisKey,node_label_2])


    return largeGraph,pos


def update_dijsktra(ATC_list, graph, graphDummy, seperation, default_values):  # this function updates the graph values for the  current situation
    v_max = default_values['v_max']
    for key, value in graph.adjacency_iter():
        for inner_key, inner_value in value.items():
            # print 'Inner:',inner_value
            # Get the current density and distance of each link(key, inner_key)
            density = inner_value['density']
            distance = inner_value['distance']
            max_density = distance / seperation

            # Calculate the speeds based on density
            if density > max_density:
                speed = 0
            elif density < 0.5 * max_density: # 0.5 * max_density:  # TODO make the the 0.5 a variable that is set up in the simulator setup
                speed = v_max
            else:
                if density > 0:
                    speed = v_max * (1 - (density / max_density))
                else:
                    speed = v_max

            # Calculate the (graph weight = time_value) based on the speed and distance
            if speed > 0:
                time_value = distance / speed  # TODO this is NOT our optimal soultion!  BALANCING DILEMMA
                add_speed = True
            else:
                time_value = float('Inf')
                add_speed = False
            graph[key][inner_key]['weight'] = time_value
            graph[key][inner_key]['speed'] = speed
        update_dijkstra_dummyGraph(graph,graphDummy,default_values)
    return graph

def update_dijkstra_dummyGraph(graph,dummyGraph,default_values):
    v_turn = default_values['v_turn']
    acc_standard = default_values['acc_standard']
    dec_standard = default_values['dec_standard']
    for key, value in dummyGraph.adjacency_iter():
        for inner_key, inner_value in value.items():
            # print key,inner_key,'Inner Value',inner_value
            if dummyGraph[key][inner_key]['turn']:
                pass
                # print 'This edge has a turn'
            origEdge = dummyGraph[key][inner_key]['original_edge']

            distance = graph[origEdge[0]][origEdge[1]]['distance']
            speed = graph[origEdge[0]][origEdge[1]]['speed']

            time_for_link = time_with_turn_penalty(distance,speed,v_turn,dec_standard,acc_standard,dummyGraph[key][inner_key]['turn'])

            dummyGraph[key][inner_key]['distance'] = distance
            dummyGraph[key][inner_key]['density'] = graph[origEdge[0]][origEdge[1]]['density']
            dummyGraph[key][inner_key]['weight'] = time_for_link

def calculate_edge_values(edge,simulation_constants):
    # get simulation constants
    separation = simulation_constants['separation']
    v_max = simulation_constants['v_max']
    v_turn = simulation_constants['v_turn']
    dec_standard = simulation_constants['dec_standard']
    acc_standard = simulation_constants['acc_standard']
    flowTheory_cutoff = simulation_constants['flowTheory_cutoff']

    # get static edge values
    density = edge['density']
    distance = edge['distance']

    # calculate dynamic edge values
    max_density = distance / separation

    ## Calculate the speeds based on density
    if density > max_density:
        speed = 0
    elif density < flowTheory_cutoff * max_density: # 0.5 * max_density:  # TODO make the the 0.5 a variable that is set up in the simulator setup
        speed = v_max
    else:
        if density > 0:
            speed = v_max * (1 - (density / max_density))
        else:
            speed = v_max

    # calcualate the conditional link time
    if edge.has_key('turn'):
        turn = edge['turn']
    else:
        turn = False
    # time_for_link = time_with_turn_penalty(distance,speed,v_turn,dec_standard,acc_standard,edge['turn'])
    time_for_link = time_with_turn_penalty(distance,speed,v_turn,dec_standard,acc_standard,turn)

    # set dynamic edge values
    edge['weight'] = time_for_link
    edge['speed'] = speed

def time_with_turn_penalty(s,v,v_t,deceleration,acceleration,turn):
    if v >0:
        if turn and v>v_t:
            # calculate the dist and time for the normal speed part
            dist_c = s-(v**2-v_t**2)/(2*acceleration)-(v_t**2-v**2)/(2*deceleration)
            time_c = dist_c/v

            # calculate the time for the acceleration and deceleariotn part
            time_a = (v-v_t)/acceleration
            time_d = (v_t-v)/deceleration

            return time_a + time_c + time_d
        else:
            return s/v
    else:
        return float('inf')




