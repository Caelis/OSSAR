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
#    runwayid = rnd.choice(runway_list).id               # Random select a runway entrance of the aircraft
#    goal = []
#    print runwayid
#    for node in runway_list[runwayid].nodes:
#        x_coor = wp_database[node][1]
#        y_coor = wp_database[node][2]
#        goal.append([x_coor,y_coor])
    ATC_runway = int(rnd.choice(rw_database))           # Random select a runway entrance of the aircraft
#    ATC_runway = rnd.choice(runway_list).id           # Random select a runway for the aircraft
    x2 = float(wp_database[ATC_gate][1])                # goal x-coordinate
    y2 = float(wp_database[ATC_gate][2])                # goal y-coordinate
    speed = 30*0.5144                                   # choose starting speed
    max_speed = v_max                                   # starting value for the maximum speed
    heading = False                                     # Set an initial value for the heading
    plane_type = int(rnd.choice(data)[0])               # choose a type of aircraft
    max_acc = int(data[int(plane_type)][4])             # set maximum accelaration
    max_dcc = int(data[int(plane_type)][3])             # set maximum deccelration
    new_plane = aircraft(idnumber,plane_type,speed,max_speed,max_acc,max_dcc,x1,y1,x1,y1,x2,y2,heading,ATC_gate,ATC_runway)
    ATC_list[ATC_gate].add_plane(new_plane)
    idnumber =  idnumber + 1
    return idnumber

#loops through all ATC and appends a 
def ATC_check(ATC_list,runway_list,graph,radar_range,runway_occupance_time,dt,t,v_max):
    plane_list = []
    for atc in ATC_list:
#        if atc.id == 19:
#            print 'at time t: '
#            for plane in atc.locp:
#                print 'plane ', plane.id, 'has v ',plane.v, ' and deceleration: ', plane.deceleration
        atc.update(ATC_list,runway_list,v_max,graph,runway_occupance_time,dt,t) # check if commands for the plane are necessary and plan operation
        aircraft_list(atc,plane_list)    # check for each aircraft which other plane are within a certain(radar) range
    for plane1 in plane_list:       # loop through all planes in the simulator
        for plane2 in plane_list:   # loop through all planes to compare to planes from above loop
            if hypot((plane2.x_pos-plane1.x_pos),(plane2.y_pos-plane1.y_pos)) < radar_range and plane1.id != plane2.id: # When aircrafts are within radarrange(exluding self):
                plane1.radar.append(plane2) # append plane to plane1.radar for check if avoidence is necessary
                #bug fixing
#                print 'plane 1: ', plane1.heading
#                print 'plane 2: ', plane2.heading
#                if plane1.atc[0] == plane2.atc[1] and plane1.atc[1] == plane2.atc[0]:
#                    print graph.edges
#                    print "opposing traffic!"

#check for each aircraft which other aircraft are within radar range
def aircraft_list(atc,plane_list):
    possible_handovers = len(atc.link)   #amount of links from which a aircraft can come
    occupied_links = []
    link_planes = []
    for plane in atc.locp:          #TODO # iterate over all planes in the simulator
        plane_list.append(plane)    # append plane to plane_list to create list of all planes for radar check
        plane.radar = []            # empty radar list of each plane
        #bug fixing
        if plane.atc[0] not in occupied_links:
            occupied_links.append(plane.atc[0])
            link_planes.append(plane)
    if len(occupied_links) == possible_handovers and possible_handovers > 1: #TODO temporary solution
        print 'deadlock at atc: ', atc.id
        print 'occupied_links: ', occupied_links
        print 'possible_handovers', possible_handovers
        for aircraft in link_planes:
            atc.locp.remove(aircraft)

#update each aircrafts position and track simulator variables (t_stop_total)            
def update_aircraft(ATC_list,plane_speed,t_stop_total,dt,separation,v_max,t):
    for atc in ATC_list:
        for plane in atc.locp:      #loop through all aircraft
            this_speed, this_stop_time = plane.update(separation,v_max,t,dt)
            plane_speed.append(this_speed)
            t_stop_total = t_stop_total + this_stop_time
    return t_stop_total, plane_speed