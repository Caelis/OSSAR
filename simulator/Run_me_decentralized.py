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
from SIM.simulator_decentralized import *

'''Configuring the simulator'''
Map = False# Activate or deactivate the map
runs = 1                # number of runs 
spawnrate = [1,20,40,60,80,100,120,140,160,180,200,220,240]       # rate [aircraft/hour] at which aircraft are added
# spawnrate = [120]       # rate [aircraft/hour] at which aircraft are added
n_prop = [0]            # degree of propagation
t_simulated = 3600      # simulation time [s]
dt = 0.5                # timestep [s]
runway_throughput = 120 # rate[aircraft/hour] at which aircraft can take-off/land
min_num_trials = 100    # minimum number of trial runs
# min_num_trials = 1

area = 30               # airspace area
marge = 0.1             # stop criteria for accuracy purposes
Za = 1.96               # stop criteria for accuracy purposes

throughput_list = []         # measurements
t_stop_total_list = []       # measurements
v_average_list = []          # measurements

for i in range(len(spawnrate)):
    for j in range(len(n_prop)):
        n_runs = []
        
        #Looping
        looping = True
        trial = 0
#        f = open("results_" + str(spawnrate[i]) +"_"+ str(n[j]) + "_"+str(Sim_type)+".txt","w")

        while looping == True:
#            simrun(t_simulated,area,dt,Map,n_prop)
            throughput,t_stop_total,v_average,position_array,edge_array = simrun(t_simulated,area,dt,Map,n_prop,runway_throughput,spawnrate[i])
            throughput_list.append(throughput)
            t_stop_total_list.append(t_stop_total)
            v_average_list.append(v_average)
            print "total stopping time: ",t_stop_total,' [s]'
            print "throughput: ", throughput
            print "v_average: ",v_average,' [m/s]'
            
            trial = trial + 1
            print "run",trial," finished..."

            filename_aircraft_pos = 'aircraftPos_' + str(spawnrate[i]) + '_' + str(trial) + '.csv'
            # filename_edge_value = 'edgeValue_' + str(spawnrate[i]) + '_' + str(trial) + '.csv'


            write_data(filename_aircraft_pos,position_array)
            # write_data(filename_edge_value,edge_array)

            # if trial == 1:   # this loop overwrites the next loop and is to make sure the simulator is only run once for testing purposes
            #     looping = False # when this loop is removed, the simulator only stops when the stop criteria is reached or forced to stop

            if trial == min_num_trials:
                looping = False

            # if len(v_average_list)>=100: # This loop determines when the simulator should stop running.
            #     average1 = np.mean(throughput_list)
            #     average2 = np.mean(t_stop_total_list)
            #     average3 = np.mean(v_average_list)
            #     d1 = marge * average1
            #     d2 = marge * average2
            #     d3 = marge * average3
            #     S1 = np.std(throughput_list)
            #     S2 = np.std(t_stop_total_list)
            #     S3 = np.std(v_average_list)
            #     if 2 *Za * S1 /np.sqrt(trial) < d1 and 2 *Za * S2 /np.sqrt(trial) < d2 and 2 *Za * S3 /np.sqrt(trial) :
            #         looping = False
#        f.close()
