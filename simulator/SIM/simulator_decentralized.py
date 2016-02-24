'''
Main simulator

description: The Main simulator simulates airtraffic in an predifined airspace for a set amount of time. Depending on the added packages the ouput can differ.

Input: Fleet package, ATC_class package, Map package

'''

#import packages
from Fleet_decentralized import *
# from ATC_class import ATC
from ATC_class_decentralized import create_ATC
# from Map import *
# from dijkstra_structure_decentralized import *
from dijkstra_structure_decentralized import *
from runway_class import *

#import python modules
from numpy import *
# from math import *
# import pygame as pg
import networkx as nx
import copy

#import data
from data_import import wpl_database
from data_export import *
def simrun(t_sim,area,dt,Map,n_prop,runway_throughput,spawnrate):
    #initializing values
    idnumber = 0
    idnumber_rw = 0
    t = 0
    t_stop_total = 0
    ATC_list = []
    runway_list = []
    plane_speed = [] 
    throughput = 0
    t_next_aircraft = 0    
    running = True
    create = True

    #properties    
    r = int(1000.0 * np.sqrt(area/np.pi))   #creating the radius of the airspace
    v_max = 30*0.5144
    separation = 250
    radar_range = 250
    mean = 3600/spawnrate #mean of aircraft spawning time
    std = 1 #standerd deviation of aircraft spawning time
    runway_occupance_time = 3600/runway_throughput #time until the next aircraft can take-off/land
    
    #initiating the simulator
    if Map == True:
        reso, scr, scrrect, plane_pic, piclist, X_waypoint, Y_waypoint = map_initialization(wp_database)

    # initiate the Dijksta algorithm
    taxiwayGraph0 = initiate_dijkstra(v_max)

    #simulator loop
    taxiwayGraphCurrent = nx.DiGraph(taxiwayGraph0)
    # print taxiwayGraph
    taxiwayGraphOriginal = copy.deepcopy(taxiwayGraphCurrent)

    # create ATC for each waypoint
    ATC_list = create_ATC(wp_database,ATC_list,taxiwayGraphCurrent)

    # create an empty list of aircraft
    aircraft_list = []
    # create an empty list of aircraft that have been removed
    inactive_aircraft_list = []


    #create all runways
    idnumber_rw, runway_list = create_runway(idnumber_rw,ATC_list,runway_list,runway_occupance_time)
    

    # print taxiwayGraphOriginal

    # Build a matrix that hold the ATCs who can access a link
    linkAtcRelations = {}
    for key, value in taxiwayGraphOriginal.adjacency_iter():
        # print key
        # linkAtcRelations.append(key)
        linkAtcRelations[key] = {}
        for inner_key, inner_value in value.items():
            # print inner_key
            # linkAtcRelations[key].append(inner_key)
            linkAtcRelations[key][inner_key] = []

    maxDistance = 2
    paths = nx.all_pairs_shortest_path(taxiwayGraphOriginal) # get all shortest paths between pairs
    for source in paths:
        # print source,':'
        for target in paths[source]:
            if len(paths[source][target]) <= maxDistance and len(paths[source][target]) > 1:
                for onePath in nx.all_shortest_paths(taxiwayGraphOriginal,source,target):
                    for indexInPath in range(0, len(onePath)-1):
                        if not ATC_list[source] in linkAtcRelations[onePath[indexInPath]][onePath[indexInPath+1]]:
                            linkAtcRelations[onePath[indexInPath]][onePath[indexInPath+1]].append(ATC_list[source])
                        if not ATC_list[target] in linkAtcRelations[onePath[indexInPath]][onePath[indexInPath+1]]:
                            linkAtcRelations[onePath[indexInPath]][onePath[indexInPath+1]].append(ATC_list[target])
                            # linkAtcRelations[onePath[indexInPath+1]][onePath[indexInPath]].append(ATC_list[source])
    # for source in linkAtcRelations:
    #     for target in linkAtcRelations[source]:
    #         # print 'Link',source,target,'has',len(linkAtcRelations[source][target]),'ATCs'
    #         for atc in linkAtcRelations[source][target]:
    #             # print atc.id

                            # print [onePath[indexInPath],onePath[indexInPath+1]]
    # Get ALL shortest paths for a pair of nodes
    # print([p for p in nx.all_shortest_paths(taxiwayGraph,source=2,target=23)])



    # perpare for export
    position_array = []
    edges_array = []
    while running == True:
        # print 'Time is: ' + str(t)
        # time.sleep(5)
        # taxiwayGraph = nx.DiGraph(taxiwayGraph0)

        #update Dijkstra structure based on current traffic situation
        taxiwayGraphCurrent = update_dijsktra(ATC_list,linkAtcRelations,taxiwayGraphCurrent,separation,v_max)
        # taxiwayGraphCurrent = update_dijsktra(ATC_list,taxiwayGraphCurrent,separation,v_max)

        #update runways
        update_runway(runway_list,runway_occupance_time,ATC_list,dt)

        # update plane positions
        update_all_aircraft_position(aircraft_list,dt)

        # update ATC (decision making here)
        taxiwayGraphCurrent = update_all_ATC_decentralized(ATC_list,runway_list,linkAtcRelations,taxiwayGraphCurrent,radar_range,runway_occupance_time,dt,t,v_max)


        # update aircraft (decision making)
        t_stop_total,plane_speed = update_aircraft(aircraft_list,plane_speed,t_stop_total,dt,separation,v_max,radar_range,t)
        ## update radar of all ac
        ## each aircraft
        ### conflict detection
        ### speed decision
        ### heading decision

        # add aircraft
        t_next_aircraft, create, idnumber = aircraft_interval(t_next_aircraft,idnumber,ATC_list,aircraft_list,runway_list,r,v_max,create,mean,std,taxiwayGraphOriginal,taxiwayGraphCurrent,taxiwayGraphCurrent,t,dt)

        # store aircraft position before removing inactive aircraft
        collect_data(position_array,'aircraft_position',aircraft_list,t)
        # collect_data(edges_array,'edge_values',taxiwayGraphCurrent,t)


        # remove aircraft that took off
        remove_inactive_aircraft(aircraft_list,inactive_aircraft_list)

        # print 'Active: ',len(aircraft_list),' Inactive: ',len(inactive_aircraft_list)

        #if True, run map
        if Map == True:
            running = map_running(reso,scr,scrrect,plane_pic,piclist,ATC_list,rectlist,running,r,X_waypoint,Y_waypoint,wp_database,wpl_database, taxiwayGraphCurrent)
        if t>= t_sim:
            running = False

        t = t + dt # update clock


    if Map == True:
        pg.quit()

    v_average = sum(plane_speed)/len(plane_speed)

    return throughput,t_stop_total,v_average,position_array,edges_array
