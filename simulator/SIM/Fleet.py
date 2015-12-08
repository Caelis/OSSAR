'''
Package: main
Module: fleet
module dependence: aircraft_class, flightplan_class

description: The fleet package manages the aircraft

Input:
Output:
'''

#import modules
from aircraft_class import *
from flightplan_class import *
from ATC_class import ATC
import random as rnd
import numpy as np

#import data
from data_import import wp_database
from data_import import ar_database
from data_import import g_database
from data_import import rw_database
from data_import import data

#Uses a normal distribution to determine when the next aircaft will arrive
def aircraft_interval(t_next_aircraft,idnumber,ATC_list,runway_list,r,v_max,create,mean,std,t,dt):
    if create == True:
        idnumber = create_aircraft(idnumber,ATC_list,runway_list,r,v_max,t,dt)
        t_next_aircraft = t + np.random.normal(mean,std)
        create = False
    if create == False:
        if t_next_aircraft <= t:
            create = True
    return t_next_aircraft,create,idnumber

def create_aircraft(idnumber,ATC_list,runway_list,r,v_max,t,dt): #creates aircraft when nessecary
    ATC_gate = int(rnd.choice(g_database))              # Random select a departure gate of the aircraft
    x1 = float(wp_database[ATC_gate][1])                # starting x-coordinate
    y1 = float(wp_database[ATC_gate][2])                # starting y-coordinate
#    runwayid = rnd.choice(runway_list).id             # Random select a runway entrance of the aircraft
#    goal = []
#    print runwayid
#    for node in runway_list[runwayid].nodes:
#        x_coor = wp_database[node][1]
#        y_coor = wp_database[node][2]
#        goal.append([x_coor,y_coor])
    ATC_runway = int(rnd.choice(rw_database))           # Random select a runway entrance of the aircraft
    x2 = float(wp_database[ATC_gate][1])                # goal x-coordinate
    y2 = float(wp_database[ATC_gate][2])                # goal y-coordinate
    speed = 30*0.5144                                    # choose starting speed
    max_speed = v_max                                   # starting value for the maximum speed
    heading = False                                     # Set an initial value for the heading
    plane_type = int(rnd.choice(data)[0])                  # choose a type of aircraft
    max_acc = int(data[int(plane_type)][4])             # set maximum accelaration
    max_dcc = int(data[int(plane_type)][3])             # set maximum deccelration
    new_plane = aircraft(idnumber,plane_type,speed,max_speed,max_acc,max_dcc,x1,y1,x1,y1,x2,y2,heading,ATC_gate,ATC_runway)
    ATC_list[ATC_gate].add_plane(new_plane)
    idnumber =  idnumber + 1
    return idnumber

#def create_aircraft(idnumber,ATC_list,r,v_max,t,dt): #creates aircraft when nessecary
#    for i in xrange(len(ar_database)):               # add aircraft if needed
#        if int(ar_database[i][-1]) >= (t - 0.5*dt) and int(ar_database[i][-1]) < (t + 0.5*dt):
#            ATC_gate = int(rnd.choice(g_database))              # Random select a departure gate of the aircraft
#            x1 = float(wp_database[ATC_gate][1])                # starting x-coordinate
#            y1 = float(wp_database[ATC_gate][2])                # starting y-coordinate
#            ATC_runway = int(rnd.choice(rw_database))           # Random select a runway entrance of the aircraft
#            x2 = float(wp_database[ATC_gate][1])                # goal x-coordinate
#            y2 = float(wp_database[ATC_gate][2])                # goal y-coordinate
#            speed = float(ar_database[i][1])*0.5144             # choose starting speed
#            max_speed = v_max                                   # starting value for the maximum speed
#            heading = False                                     # Set an initial value for the heading
#            plane_type = ar_database[i][0]                      # choose a type of aircraft
#            max_acc = int(data[int(plane_type)][4])             # set maximum accelaration
#            max_dcc = int(data[int(plane_type)][3])             # set maximum deccelration
#            new_plane = aircraft(idnumber,plane_type,speed,max_speed,max_acc,max_dcc,x1,y1,x1,y1,x2,y2,heading,ATC_gate,ATC_runway)
#            ATC_list[ATC_gate].add_plane(new_plane)
#            idnumber =  idnumber + 1

def ATC_check(ATC_list,runway_list,graph,dt,t,v_max):
    for atc in ATC_list:
        atc.command_check(ATC_list,runway_list,v_max,graph,dt,t)

def execute_commands(ATC_list,separation,v_max,t,dt):
    for atc in ATC_list:
        for plane in atc.locp:
            plane.decision_making(ATC_list,separation,v_max,dt)
            
def update_aircraft(ATC_list,plane_speed,t_stop_total,dt):
    for atc in ATC_list:
        for plane in atc.locp:      #loop through all aircraft
            plane.update_pos(dt)    #update the position of each aircraft
            if plane.v < 0.5:       #each time step calculate the total stopping time
                t_stop_total = t_stop_total + dt
            plane_speed.append(plane.v) #list current aircraft speed for further calculations
    return t_stop_total, plane_speed