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
from Dijkstra_class import *
from runway_class import *

#import python modules
from numpy import *
from math import *
import pygame as pg
from copy import deepcopy

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
    separation = 100
    radar_range = 250
    mean = 3600/spawnrate #mean of aircraft spawning time
    std = 1 #standerd deviation of aircraft spawning time
    
    #initiating the simulator
    if Map == True:
        reso, scr, scrrect, plane_pic, piclist, X_waypoint, Y_waypoint = map_initialization(wp_database)
        
    # create ATC for each waypoint
    ATC_list = create_ATC(wp_database,ATC_list)

    #create all runways
    idnumber_rw, runway_list = create_runway(idnumber_rw,runway_list,runway_throughput)    
    
    # initiate the Dijksta algorithm
    structure_orig, struc_dist, struc_dens = initiate_dijkstra(v_max)
    struc_dens0 = deepcopy(struc_dens)
    structure = structure_orig.copy()

    #simulator loop    
    while running == True:
#        print "t: "+str(t)
        #create new aircraft if nessecary
        t_next_aircraft, create, idnumber = aircraft_interval(t_next_aircraft,idnumber,ATC_list,runway_list,r,v_max,create,mean,std,t,dt)
        #create and execute commands
        ATC_check(ATC_list,runway_list,structure,radar_range,dt,t,v_max) # check for new commands from the ATC
        execute_commands(ATC_list,separation,v_max,t,dt) # excecute all commands
        #update the aicraft position
        t_stop_total,plane_speed = update_aircraft(ATC_list,plane_speed,t_stop_total,dt) 
        #update Dijkstra structure
#        structure = update_dijsktra(ATC_list,structure_orig,struc_dens0,struc_dist,separation,v_max)
        #is True, run map
        if Map == True:
            running = map_running(reso,scr,scrrect,plane_pic,piclist,ATC_list,rectlist,running,r,X_waypoint,Y_waypoint,wp_database)
        if t>= t_sim:
            running = False

        t = t + dt # update clock
    if Map == True:
        pg.quit()
    
    v_average = sum(plane_speed)/len(plane_speed)

    return throughput,t_stop_total,v_average 
