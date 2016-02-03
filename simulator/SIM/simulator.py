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
from numpy import *
from math import *
import pygame as pg
import networkx as nx

#import data
from data_import import wpl_database

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

    # create ATC for each waypoint
    ATC_list = create_ATC(wp_database,ATC_list)

    # create an empty list of aircraft
    aircraft_list = []
    # create an empty list of aircraft that have been removed
    aircraft_list_removed = []


    #create all runways
    idnumber_rw, runway_list = create_runway(idnumber_rw,ATC_list,runway_list)    
    
    # initiate the Dijksta algorithm
    taxiwayGraph0 = initiate_dijkstra(v_max)

    #simulator loop
    taxiwayGraph = nx.DiGraph(taxiwayGraph0)
    while running == True:
        # taxiwayGraph = nx.DiGraph(taxiwayGraph0)

        #update Dijkstra structure based on current traffic situation
        taxiwayGraph = update_dijsktra(ATC_list,taxiwayGraph,separation,v_max)

        #update runways
        update_runway(runway_list,runway_occupance_time,ATC_list,dt)

        # add aircraft
        t_next_aircraft, create, idnumber = aircraft_interval(t_next_aircraft,idnumber,ATC_list,aircraft_list,runway_list,r,v_max,create,mean,std,taxiwayGraph,t,dt)

        # update plane positions
        update_all_aircraft_position(aircraft_list,dt)

        # update ATC (decision making here)
        taxiwayGraph = ATC_check(ATC_list,runway_list,taxiwayGraph,radar_range,runway_occupance_time,dt,t,v_max)

        ## radar (distance of aircraft from ATC)
        ## handoff decision
        ## graph update
        ## heading and speed commands


        # update aircraft (decision making)
        ## update radar of all ac
        ## each aircraft
        ### conflict detection
        ### speed decision
        ### heading decision

        #create new aircraft if nessecary
        #create and execute commands
        # excecute all commands
        # execute_commands(ATC_list,separation,v_max,t,dt)
        #update the aicraft position
        t_stop_total,plane_speed = update_aircraft(aircraft_list,plane_speed,t_stop_total,dt,separation,v_max,radar_range,t)
        #if True, run map
        if Map == True:
            running = map_running(reso,scr,scrrect,plane_pic,piclist,ATC_list,rectlist,running,r,X_waypoint,Y_waypoint,wp_database,wpl_database, taxiwayGraph)
        if t>= t_sim:
            running = False

        t = t + dt # update clock
    if Map == True:
        pg.quit()

    v_average = sum(plane_speed)/len(plane_speed)

    return throughput,t_stop_total,v_average
