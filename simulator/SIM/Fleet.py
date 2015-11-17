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

#import data
from data_import import wp_database
from data_import import ar_database
from data_import import g_database
from data_import import rw_database
from data_import import data

def create_aircraft(idnumber,ATC_list,r,v_max,t,dt): #creates aircraft when nessecary
    for i in xrange(len(ar_database)):               # add aircraft if needed
        if int(ar_database[i][-1]) > (t - 0.5*dt) and int(ar_database[i][-1]) < (t + 0.5*dt):
            ATC_gate = int(rnd.choice(g_database))              # Random select a departure gate of the aircraft
            x1 = float(wp_database[ATC_gate][1])                # starting x-coordinate
            y1 = float(wp_database[ATC_gate][2])                # starting y-coordinate
            ATC_runway = int(rnd.choice(rw_database))           # Random select a runway entrance of the aircraft
            x2 = float(wp_database[ATC_gate][1])                # goal x-coordinate
            y2 = float(wp_database[ATC_gate][2])                # goal y-coordinate
            speed = float(ar_database[i][1])*0.5144             # choose starting speed
            max_speed = v_max                                   # starting value for the maximum speed
            heading = False                                     # Set an initial value for the heading
            plane_type = ar_database[i][0]                      # choose a type of aircraft
            max_acc = int(data[int(plane_type)][4])             # set maximum accelaration
            max_dcc = int(data[int(plane_type)][3])             # set maximum deccelration
            new_plane = aircraft(idnumber,plane_type,speed,max_speed,max_acc,max_dcc,x1,y1,x1,y1,x2,y2,heading,ATC_gate,ATC_runway)
            ATC_list[ATC_gate].add_plane(new_plane)
            idnumber =  idnumber + 1
#            planes[listnumber].flightplan = flightplan_ins.create_flightplan(listnumber+1,planes[listnumber].x_beg,planes[listnumber].y_beg,planes[listnumber].x_des,planes[listnumber].y_des,wp_database,r) #add a flightplan to the aircraft
#            listnumber = listnumber + 1

def ATC_check(ATC_list,dijk,dt,t,v_max):
    for atc in ATC_list:
        atc.create_commands(ATC_list,v_max,dijk,dt,t)

def execute_commands(ATC_list,v_max,t,dt):
    for atc in ATC_list:
        for plane in atc.locp:
            plane.execute(v_max,dt)
            
def update_aircraft(ATC_list,dt):
    for atc in ATC_list:
        for plane in atc.locp:       #loop through all aircraft
            plane.update_pos(dt)    #update the position of each aircraft