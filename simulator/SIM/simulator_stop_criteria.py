'''
Package:
Module:
module dependence:

description:

Input:
Output:
'''

import numpy as np

def simulator_stop_criteria(position_array,v_average_list,stop_type_list,trial,min_num_runs,marge,Za):
    stop_criteria = []
    looping = True
    
    v_average_list = append_v_average_list(position_array,v_average_list)
    stop_type_list = append_stop_type_list(stop_type_list,position_array,stop_criteria,trial,marge,Za)

    if len(v_average_list) >= min_num_runs: # This loop determines when the simulator should stop running.
        stop_criteria = check_average_speed(v_average_list,stop_criteria,trial,marge,Za) #determine if avarge speeds are reliable
        stop_criteria = check_plane_stop(stop_type_list,stop_criteria,trial,marge,Za)
        
    #check if all stop criteria are true, if yes stop simulator, else continue
    print stop_criteria
    for criterion in stop_criteria:
        if not criterion:
            looping = True
            break
        else:
            looping = False
    return looping,v_average_list,stop_type_list

#appends the average taxi_speed for one simulation to v_average_list
def append_v_average_list(position_array,v_average_list):
    v_average = position_array.mean(0)[0,5] #gives the mean of the speed colunmn
    v_average_list.append(v_average)
    return v_average_list

#appends the amount of stops per stop type for one simulation to stop_type_list
def append_stop_type_list(stop_type_list,position_array,stop_criteria,trial,marge,Za):
    stop_types = {} #stop types for a single simulation
    existing_stop_types = [1,2,4,8,16,32,64,128,256,512]
    for option in existing_stop_types:
        stop_types[str(option)] = 0
    for x in xrange(len(position_array)):
        stop_type = int(position_array.item(x,-1)) #stop type for this aircraft
        stop_types = stop_types_loop(stop_type,stop_types,existing_stop_types)
    stop_type_list.append(stop_types)
    return stop_type_list

#Checks if the average speeds are within the confidence interval
def check_average_speed(v_average_list,stop_criteria,trial,marge,Za):
    mean_v_average = np.mean(v_average_list)
    std_v_average = np.std(v_average_list)
    d_v_avg = marge * mean_v_average
    if 2 *Za * std_v_average / np.sqrt(trial) < d_v_avg:
        v_avg_stop = True
    else:
        v_avg_stop = False
    stop_criteria.append(v_avg_stop) 
    return stop_criteria

#loops through all existing stop types to see if there are one or more active in each datapoint
def stop_types_loop(stop_type,stop_types,existing_stop_types):
    for option in existing_stop_types:
        if (stop_type & option) > 0:
            stop_types[str(option)] = stop_types[str(option)] + 1
    return stop_types

#checks if each stop type is within the confidence interval
def check_plane_stop(stop_type_list,stop_criteria,trial,marge,Za):
    for key in stop_type_list[0]:
        length = len(stop_type_list)
#        key_list = (option2[key] for option2 in stop_type_list)
#        print key_list
        mean = sum(option1[key] for option1 in stop_type_list) / length
        maximum = max([x[key] for x in stop_type_list])
        std = np.sqrt(sum(abs(mean - np.array([option2[key] for option2 in stop_type_list]))) / length)
        d = marge * mean
        if maximum != 0:
            if 2 *Za * std / np.sqrt(trial) < d:
                key_stop = True
            else:
                key_stop = False
        else:
            key_stop = True
        stop_criteria.append(key_stop)
    return stop_criteria        