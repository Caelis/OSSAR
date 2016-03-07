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
# from flightplan_class import *
from ATC_class import ATC
import random as rnd
import numpy as np
import time

#import data
from data_import import wp_database
from data_import import ar_database
from data_import import g_database
from data_import import rw_database
from data_import import data

#Uses a normal distribution to determine when the next aircaft will arrive
def aircraft_interval(t_next_aircraft,idnumber,ATC_list,aircraft_list,runway_list,r,v_max,create,mean,std,graphDict,min_separation,t,dt):
    if create == True:
        idnumber = create_aircraft(idnumber,ATC_list,aircraft_list,runway_list,r,v_max,graphDict,min_separation,t,dt)
        t_next_aircraft = t + np.random.normal(mean,std)
        create = False
    if create == False:
        if t_next_aircraft <= t:
            create = True
    return t_next_aircraft,create,idnumber

# def add_random_aircraft_at_rate(rate,idnumber,ATC_list,aircraft_list,runway_list,r,v_max,graph,t,dt):
#     randNumber = random.random()
#     if randNumber <= rate/float(60)/float(60)*dt:
#         idnumber = create_aircraft(idnumber,ATC_list,aircraft_list,runway_list,r,v_max,graph,t,dt)
#         return idnumber
#     else:
#         return False

def check_if_gate_available(min_separaion,ATC_list,ATC_gate,graphDict):
    graph = graphDict['graph']
    source = ATC_gate
    # print source
    target = ATC_list[source].link[0][1]
    # print target

    link_distance = graph[source][target]['distance']

    closest_ac_pos = 0
    for aircraft in ATC_list[target].locp:
        if aircraft.atc[0] == source:
            if aircraft.distance_to_atc > closest_ac_pos:
                closest_ac_pos = aircraft.distance_to_atc

    distance_closets_aircraft = link_distance - closest_ac_pos

    if distance_closets_aircraft >= min_separaion:
        return True
    else:
        # print link_distance,' - ',closest_ac_pos,'(',ATC_list[target].locp[-1].id,') = ',distance_closets_aircraft
        return False

def create_aircraft(idnumber,ATC_list,aircraft_list,runway_list,r,v_max,graphDict,min_separation,t,dt): #creates aircraft when nessecary
    # select origin and destination
    ATC_gate = int(rnd.choice(g_database))              # Random select a departure gate of the aircraft
    ATC_runway = int(rnd.choice(rw_database))           # Random select a runway entrance of the aircraft

    gate_available = check_if_gate_available(min_separation,ATC_list,ATC_gate,graphDict)

    # initilaize coordinates
    x1 = float(wp_database[ATC_gate][1])                # starting x-coordinate
    y1 = float(wp_database[ATC_gate][2])                # starting y-coordinate
    x2 = float(wp_database[ATC_gate][1])                # goal x-coordinate
    y2 = float(wp_database[ATC_gate][2])                # goal y-coordinate

    # Initialize plane tyep
    plane_type = int(rnd.choice(data)[0])               # choose a type of aircraft
    max_acc = int(data[int(plane_type)][4])             # set maximum accelaration
    max_dcc = int(data[int(plane_type)][3])             # set maximum deccelration

    # initialize speed and heading
    speed = v_max                                   # choose starting speed
    max_speed = v_max                                   # starting value for the maximum speed
    heading = 1.5                                     # Set an initial value for the heading

    # create a new plane && add to aircraft list
    new_plane = aircraft(idnumber,plane_type,speed,max_speed,max_acc,max_dcc,x1,y1,x1,y1,x2,y2,heading,ATC_gate,ATC_runway)
    aircraft_list.append(new_plane)

    if gate_available:
        ATC_list[ATC_gate].add_plane(new_plane)
    else:
        # print new_plane.id,'was not added at gate',ATC_gate,'!'
        # ATC_list[ATC_gate].add_plane(new_plane)
        new_plane.stop = 512
        new_plane.is_active = False
    # # only plan operation to next atc, as there is only one path from the gate
    # new_plane.atc_goal = ATC_list[ATC_gate].link[0][1]
    #
    # ATC_list[ATC_gate].plan_operation(new_plane,graph,t)
    #
    # new_plane.atc_goal = ATC_runway
    # new_plane.heading = new_plane.heading+new_plane.op[0].par['turn_angle']
    # new_plane.atc = [ATC_gate,new_plane.op[0].par['next_atc']]

    idnumber =  idnumber + 1
    return idnumber

#loops through all ATC and appends a
def update_all_ATC(ATC_list,runway_list,graphDict,radar_range,runway_occupance_time,dt,t,v_max,simulation_constants):
    graph = graphDict['graph']
    ## each atc
    for atc in ATC_list:
        ### radar (distance of aircraft from ATC)
        ### handoff decision
        ### graph update
        ### heading and speed commands
        graph = atc.update(ATC_list,runway_list,v_max,graphDict,runway_occupance_time,dt,t,simulation_constants) # check if commands for the plane are necessary and plan operation
        # aircraft_radar_list(atc)    # check for each aircraft which other plane are within a certain(radar) range
    return graph

#check for each aircraft which other aircraft are within radar range
def aircraft_radar_list(atc):
    possible_handovers = len(atc.link)   #amount of links from which a aircraft can come
    occupied_links = []
    link_planes = []
    for plane in atc.locp:          #TODO # iterate over all planes in the simulator
        if plane.atc[0] not in occupied_links:
            occupied_links.append(plane.atc[0])
            link_planes.append(plane)
    if len(occupied_links) == possible_handovers and possible_handovers > 1: #TODO temporary solution
        # print 'deadlock at atc: ', atc.id
        # print 'occupied_links: ', occupied_links
        # print 'possible_handovers', possible_handovers
        for aircraft in link_planes:
            atc.locp.remove(aircraft)

#update each aircrafts position and track simulator variables (t_stop_total)
def update_aircraft(aircraft_list,plane_speed,t_stop_total,dt,separation,v_max,radar_range,t):
    update_all_aircraft_radar(aircraft_list,radar_range)
    for plane in aircraft_list:      #loop through all aircraft
        this_speed, this_stop_time = plane.update(separation,v_max,t,dt)
        # print 'new_speed = ',plane.v,' - (',dt,'*',plane.deceleration,')'
        plane_speed.append(this_speed)
        t_stop_total = t_stop_total + this_stop_time
    return t_stop_total, plane_speed

def update_all_aircraft_position(aircraft_list,dt):
    for thisAircraft in aircraft_list:
        # if thisAircraft.stop & 8:
        #     # print str(thisAircraft.id) + ' is stopped with: ' + str(thisAircraft.stop)
        #     print str(thisAircraft.id) + ' from: ' + str(thisAircraft.atc[0]) + ' to: ' + str(thisAircraft.atc_goal)
        # thisAircraft.update_speed(dt)
        # thisAircraft.update_pos(dt)
        thisAircraft.update_pos_speed(dt)

def update_all_aircraft_radar(plane_list,radar_range):
    for plane1 in plane_list:       # loop through all planes in the simulator
        plane1.radar = []
        for plane2 in plane_list:   # loop through all planes to compare to planes from above loop
            if hypot((plane2.x_pos-plane1.x_pos),(plane2.y_pos-plane1.y_pos)) < radar_range and plane1.id != plane2.id: # When aircrafts are within radarrange(exluding self):
                plane1.radar.append(plane2) # append plane to plane1.radar for check if avoidence is necessary


def remove_inactive_aircraft(aircraft_list,inactive_aircraft_list):
    for aircraft in aircraft_list:
        if not aircraft.is_active:
            inactive_aircraft_list.append(aircraft)
            aircraft_list.remove(aircraft)

