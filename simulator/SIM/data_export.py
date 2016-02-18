'''
Package: main
Module: fleet
module dependence: aircraft_class, flightplan_class

description: The fleet package manages the aircraft

Input:
Output:
'''
import csv


def collect_data(export_array,type,data,t):
    if type == 'aircraft_position':
        for aircraft in data:
            export_array.append([t,aircraft.id,aircraft.x_pos, aircraft.y_pos,aircraft.heading,aircraft.v,aircraft.deceleration,aircraft.stop])
    elif type == 'edge_values':
        for edge in data.edges_iter(data='weight', default=1):
            # print [t,edge[0],edge[1],edge[2]]
            export_array.append([t,edge[0],edge[1],edge[2]])



def write_data(filename,export_array):
    with open(filename, "wb") as f:
        writer = csv.writer(f)
        writer.writerows(export_array)