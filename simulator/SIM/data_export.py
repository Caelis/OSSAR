'''
Package: main
Module: fleet
module dependence: aircraft_class, flightplan_class

description: The fleet package manages the aircraft

Input:
Output:
'''
import csv


def collect_data(export_array,type,aircraft_list,t):
    for aircraft in aircraft_list:
        export_array.append([t,aircraft.id,aircraft.x_pos, aircraft.y_pos,aircraft.heading,aircraft.v,aircraft.deceleration,aircraft.stop])


def write_data(filename,type,export_array):
    with open(filename, "wb") as f:
        writer = csv.writer(f)
        writer.writerows(export_array)