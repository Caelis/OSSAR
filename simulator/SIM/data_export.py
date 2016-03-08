'''
Package:
Module:
module dependence:

description:

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

def compile_averages_data(averages_data,stop_type_list):
    # print averages_data
    return_array = []
    for index in range(0,len(averages_data['speed'])):
        thisRow = []
        thisRow.append(averages_data['speed'][index])
        thisRow.append(averages_data['avg_taxi_time'][index])
        thisRow.append(averages_data['plane_stops'][index])
        for stop_type in stop_type_list:
            thisRow.append(averages_data['stop_types'][index][str(stop_type)])
        return_array.append(thisRow)
    return return_array
