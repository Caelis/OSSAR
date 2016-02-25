'''
Package:
Module:
module dependence:
description:
Input:
Output:
'''

import numpy as np

def simulator_stop_criteria(v_average_list,stop_type_list,trial,min_num_trials,marge,Za):
    stop_criteria = []
    looping = True

    if len(v_average_list) >= min_num_trials: # This loop determines when the simulator should stop running.
        stop_criteria = check_average_speed(v_average_list,stop_criteria,trial,marge,Za,min_num_trials) #determine if avarge speeds are reliable
        stop_criteria = check_plane_stop(stop_type_list,stop_criteria,trial,marge,Za,min_num_trials)

    #check if all stop criteria are true, if yes stop simulator, else continue
    print stop_criteria
    for criterion in stop_criteria:
        if not criterion:
            looping = True
            break
        else:
            looping = False
    return looping

#Checks if the average speeds are within the confidence interval
def check_average_speed(v_average_list, stop_criteria, trial, marge, Za, min_num_before_filter):
    this_v_average_list = np.array(v_average_list)

    if trial>min_num_before_filter:
        this_v_average_list = reject_outliers(this_v_average_list)

    mean_v_average = np.mean(this_v_average_list)
    std_v_average = np.std(this_v_average_list, ddof = 1)

    ### HEIKO
    if abs(mean_v_average) < 1:
        d_v_avg = marge
    else:
        d_v_avg = marge*mean_v_average
    ###
    # d_v_avg = marge * mean_v_average
    if 2 *Za * std_v_average / np.sqrt(trial) < d_v_avg:
        v_avg_stop = True
    else:
        v_avg_stop = False
    stop_criteria.append(v_avg_stop)
    return stop_criteria

#checks if each stop type is within the confidence interval
def check_plane_stop(stop_type_list,stop_criteria,trial,marge,Za,min_num_before_filter):
    for stop_type_column in stop_type_list[0]:

        this_stop_type_list = []
        for row in stop_type_list:
            this_stop_type_list.append(row[stop_type_column])
        # this_stop_type_list = zip(*stop_type_list)[stop_type_column]

        this_stop_type_list = np.array(this_stop_type_list)

        if trial>min_num_before_filter:
            this_stop_type_list = reject_outliers(this_stop_type_list)

        mean = np.mean(this_stop_type_list)

        maximum = np.max(this_stop_type_list)

        std = np.std(this_stop_type_list, ddof = 1)

        ### HEIKO
        if abs(mean) < 1:
            d = marge
        else:
            d = marge * mean
        ###
        # d = marge * mean
        if maximum != 0:
            if 2 *Za * std / np.sqrt(trial) < d:
                key_stop = True
            else:
                key_stop = False
        else:
            key_stop = True
        stop_criteria.append(key_stop)
    return stop_criteria

# not used this http://stackoverflow.com/questions/22354094/pythonic-way-of-detecting-outliers-in-one-dimensional-observation-data
def reject_outliers(data, m = 2.):
    #http://stackoverflow.com/questions/11686720/is-there-a-numpy-builtin-to-reject-outliers-from-a-list
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d/mdev if mdev else 0.
    return data[s<m]