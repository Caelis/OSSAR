'''
Package:
Module:
module dependence:
description:
Input:
Output:
'''

import numpy as np

def append_to_result_lists(position_array,v_average_list,stop_type_list):
    v_average_list = append_v_average_list(position_array,v_average_list)
    stop_type_list = append_stop_type_list(stop_type_list,position_array)
    return v_average_list,stop_type_list

############
## Append Funcitons
############
#appends the average taxi_speed for one simulation to v_average_list
def append_v_average_list(position_array,v_average_list):
    v_average = position_array.mean(0)[0,5] #gives the mean of the speed colunmn
    v_average_list.append(v_average)
    return v_average_list

#appends the amount of stops per stop type for one simulation to stop_type_list
def append_stop_type_list(stop_type_list,position_array): #,stop_criteria,trial,marge,Za):
    stop_types = {} #stop types for a single simulation
    existing_stop_types = [1,2,4,8,16,32,64,128,256,512]
    for option in existing_stop_types:
        stop_types[str(option)] = 0
    for x in xrange(len(position_array)):
        # TODO zip(*position_array)[-1]
        stop_type = int(position_array.item(x,-1)) #stop type for this aircraft
        stop_types = stop_types_loop(stop_type,stop_types,existing_stop_types)
    stop_type_list.append(stop_types)
    return stop_type_list

#loops through all existing stop types to see if there are one or more active in each datapoint
def stop_types_loop(stop_type,stop_types,existing_stop_types):
    for option in existing_stop_types:
        if (stop_type & option) > 0:
            stop_types[str(option)] = stop_types[str(option)] + 1
    return stop_types