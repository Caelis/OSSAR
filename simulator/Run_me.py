'''
Run_me

In this script the experiment setup can be determined, which can depend on several factors:
Map: The map gives a graphical display of the aircraft. For testing purposes the map is turned off, to increase simulator speed.
 runs: this sets the amount of runs for the whole simulator
t_simulated: The equivalent time the simulator runs in seconds.
dt: timestep of the simulator in seconds. When using the map it is recommended to use a value between 0.1 and 2.0.
area: This sets the area of the airspace. This can have multiple values when testing at multiple setups, but should have at least the same list length as max_number.
marge and Za: These are values to determine the stop criteria.
test'''
Sim_type = '1wpbeg-end-comb'

#import python modules
import os
# import numpy as np
# import scipy.stats as stats
# import matplotlib.pyplot as plt
#import the simulator
from SIM.simulator import *
from SIM.simulator_stop_criteria import *
from SIM.process_results import *

'''Configuring the simulator'''
Map = True# Activate or deactivate the map
runs = 1                # number of runs
# spawnrate = [1,20,40,60,80,100,120,140,160,180,200,220,240]       # rate [aircraft/hour] at which aircraft are added
spawnrate = [129]
n_prop = [0]            # degree of propagation
t_simulated = 3600      # simulation time [s]
dt = 0.5              # timestep [s]
runway_throughput = 120 # rate[aircraft/hour] at which aircraft can take-off/land
min_num_trials = 100    # minimum number of trial runs
max_num_trials = 500

area = 30               # airspace area
marge = 0.1             # stop criteria for accuracy purposes
Za = 1.96               # stop criteria for accuracy purposes

sim_params = {}
sim_params['t_sim'] = t_simulated
sim_params['area'] = area
sim_params['dt'] = dt
sim_params['Map'] = Map
sim_params['n_prop'] = n_prop
sim_params['runway_throughput'] = runway_throughput
sim_params['flowTheory_cutoff'] = 0.5


for i in range(len(spawnrate)):
    throughput_list = []        # measurements
    t_stop_total_list = []      # measurements
    v_average_list = []         # measurements
    stop_type_list = []         # measurements
    taxi_time_average_list = [] # measurements
    n_stop_list = []            # measurements
    origin_destination_list = []# measurements
    for j in range(len(n_prop)):
        n_runs = []
        
        #Looping
        looping = True
        trial = 0
#        f = open("results_" + str(spawnrate[i]) +"_"+ str(n[j]) + "_"+str(Sim_type)+".txt","w")

        while looping == True:
            print spawnrate[i]
            sim_params['spawnrate'] = spawnrate[i]

            throughput,t_stop_total,v_average,position_array,edge_array,flights_array,aircraft_accelerating,taxi_time_average = simrun(sim_params)

            trial = trial + 1
            print "run",trial," finished..."

            ############
            ## Write position data
            ############
            # filename_aircraft_pos = 'aircraftPos_' + str(spawnrate[i]) + '_' + str(trial) + '.csv'
            # filename_flights = 'flights_' + str(spawnrate[i]) + '_' + str(trial) + '.csv'
            # filename_edge_value = 'edgeValue_' + str(spawnrate[i]) + '_' + str(trial) + '.csv'
            # write_data(filename_aircraft_pos,position_array)
            # write_data(filename_flights,flights_array)
            # write_data(filename_edge_value,edge_array)

            # append data of this run to measurements
            position_array = np.matrix(position_array)
            # store the arrays
            v_average_list,stop_type_list = append_to_result_lists(position_array,v_average_list,stop_type_list)
            # store the averages data
            taxi_time_average_list.append(taxi_time_average)
            n_stop_list.append(aircraft_accelerating)
            origin_destination_list.append(accumulate_data(flights_array))

            looping = simulator_stop_criteria(v_average_list,taxi_time_average_list,n_stop_list,stop_type_list,trial,min_num_trials,marge,Za)

            if trial > max_num_trials:
                looping = False

            filename_averages = 'averages_' + str(spawnrate[i]) + '.csv'
            averages_data = {}
            averages_data['speed'] = v_average_list
            averages_data['stop_types'] = stop_type_list
            averages_data['plane_stops'] = n_stop_list
            averages_data['avg_taxi_time'] = taxi_time_average_list
            averages_data['origin_destination'] = origin_destination_list
            averages_array = compile_averages_data(averages_data,[1,2,4,8,16,32,64,128,256,512,1024])
            write_data(filename_averages,averages_array)
