'''
Main simulator

description: The Main simulator simulates airtraffic in an predifined airspace for a set amount of time. Depending on the added packages the ouput can differ.

Input: Fleet package, ATC_class package, Map package

'''

#import packages
from Fleet import *
from ATC_class import ATC
from ATC_class import create_ATC
from Map import *
from dijkstra_structure import *
from runway_class import *

#import python modules
import numpy as np
from math import *
import pygame as pg
import networkx as nx
import copy

#import data
from data_import import wpl_database
from data_export import *
def simrun(sim_params):

    t_sim = sim_params['t_sim']
    area = sim_params['area']
    dt = sim_params['dt']
    Map = sim_params['Map']
    n_prop = sim_params['n_prop']
    runway_throughput = sim_params['runway_throughput']
    spawnrate = sim_params['spawnrate']

    #initializing values
    idnumber = 0
    idnumber_rw = 0
    t = 0
    t_stop_total = 0
    ATC_list = []
    runway_list = []
    plane_speed = []
    taxi_times = []
    throughput = 0
    t_next_aircraft = 0
    aircraft_accelerating = 0
    running = True
    create = True

    #properties    
    r = int(1000.0 * np.sqrt(area/np.pi))   #creating the radius of the airspace
    v_max = 30*0.5144
    separation = 250
    radar_range = 250
    mean = 3600.0/spawnrate #mean of aircraft spawning time
    std = 1 #standerd deviation of aircraft spawning time
    runway_occupance_time = 3600/runway_throughput #time until the next aircraft can take-off/land

    # define default parameters for the simulation
    simulation_constants= {}
    simulation_constants['v_max'] = v_max
    simulation_constants['v_turn'] = 10*0.5144
    simulation_constants['acc_standard'] = 0.8
    simulation_constants['dec_standard'] = -2
    simulation_constants['separation'] = separation
    simulation_constants['flowTheory_cutoff'] = 0.1

    #initiating the simulator
    if Map == True:
        reso, scr, scrrect, plane_pic, piclist, X_waypoint, Y_waypoint = map_initialization(wp_database)

    # initiate the Dijksta algorithm
    taxiwayGraph0 = initiate_dijkstra(v_max)
    taxiwayGraphDummy0, pos = add_dummy_edges(taxiwayGraph0,wp_database,simulation_constants)

    #simulator loop
    # taxiwayGraph = nx.DiGraph(taxiwayGraph0)
    taxiwayGraph = taxiwayGraph0.copy()
    taxiwayGraphDummy = taxiwayGraphDummy0.copy()

    graphDict={}
    graphDict['graph'] = taxiwayGraph
    graphDict['graphOrig'] = taxiwayGraph0
    graphDict['dummyGraph'] = taxiwayGraphDummy
    graphDict['dummyGraphOrig'] = taxiwayGraphDummy0

    ATC_list = create_ATC(wp_database,ATC_list)
    # for atc in ATC_list:
    #     print atc.type

    # create an empty list of aircraft
    aircraft_list = []
    # create an empty list of aircraft that have been removed
    inactive_aircraft_list = []


    #create all runways
    idnumber_rw, runway_list = create_runway(idnumber_rw,ATC_list,runway_list,runway_occupance_time)

    # perpare for export
    position_array = []
    edges_array = []
    flights_array = []

    np.random.seed()
    while running == True:
        # print 'Time is: ' + str(t)
        # time.sleep(5)
        # taxiwayGraph = nx.DiGraph(taxiwayGraph0)

        #update Dijkstra structure based on current traffic situation
        # THIS we do now at handoff to save some computaiton time.
        # update_dijsktra(ATC_list,taxiwayGraph,taxiwayGraphDummy,separation,simulation_constants)

        #update runways
        update_runway(runway_list,runway_occupance_time,ATC_list,dt)

        # update plane positions
        aircraft_accelerating_this_step = update_all_aircraft_position(aircraft_list,dt)

        # update ATC (decision making here)
        taxiwayGraph,planes_taxi_time_this_step = update_all_ATC(ATC_list,runway_list,graphDict,radar_range,runway_occupance_time,dt,t,v_max,simulation_constants)


        # update aircraft (decision making)
        t_stop_total,plane_speed = update_aircraft(aircraft_list,plane_speed,t_stop_total,dt,separation,v_max,radar_range,t)
        ## update radar of all ac
        ## each aircraft
        ### conflict detection
        ### speed decision
        ### heading decision

        # add aircraft
        t_next_aircraft, create, idnumber = aircraft_interval(t_next_aircraft,idnumber,ATC_list,aircraft_list,flights_array,runway_list,r,v_max,create,mean,std,graphDict,separation,t,dt)

        # store aircraft position before removing inactive aircraft
        collect_data(position_array,'aircraft_position',aircraft_list,t)

        # collect_data(edges_array,'edge_values',taxiwayGraph,t)

        taxi_times = taxi_times + planes_taxi_time_this_step

        aircraft_accelerating = aircraft_accelerating + aircraft_accelerating_this_step
        # print 'Taxi times:',taxi_times
        # print 'Aircraft stops:',aircraft_accelerating
        # collect_data(edges_array,'edge_values',taxiwayGraph,t)
        # remove aircraft that took off
        remove_inactive_aircraft(aircraft_list,inactive_aircraft_list)

        # print 'Active: ',len(aircraft_list),' Inactive: ',len(inactive_aircraft_list)

        #if True, run map
        if Map == True:
            running = map_running(reso,scr,scrrect,plane_pic,piclist,ATC_list,rectlist,running,r,X_waypoint,Y_waypoint,wp_database,wpl_database, graphDict['graph'])
        if t>= t_sim:
            running = False
        t = t + dt # update clock


    if Map == True:
        pg.quit()

    v_average = sum(plane_speed)/len(plane_speed)
    taxi_time_average = 1.0*sum(taxi_times)/len(taxi_times)
    return throughput,t_stop_total,v_average,position_array,edges_array,flights_array,aircraft_accelerating,taxi_time_average
