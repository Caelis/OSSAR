'''
Run_me

In this script the experiment setup can be determined, which can depend on several factors:
Map: The map gives a graphical display of the aircraft. For testing purposes the map is turned off, to increase simulator speed.
max_number: This sets the amount of aicraft in the airspace at the same time. This can have multiple values when testing at multiple setups.
t_simulated: The equivalent time the simulator runs in seconds.
dt: timestep of the simulator in seconds. When using the map it is recommended to use a value between 0.1 and 2.0.
area: This sets the area of the airspace. This can have multiple values when testing at multiple setups, but should have at least the same list length as max_number.
marge and Za: These are values to determine the stop criteria.
test'''
Sim_type = '1wpbeg-end-comb'

#import python modules
import os
import numpy as np
import scipy.stats as stats
import matplotlib.pyplot as plt
#import the simulator
from SIM.simulator import *

'''Configuring the simulator'''
Map = True              # Activate or deactivate the map
max_number = [30]       # Number of aircraft:
t_simulated = 3000      # simulation time [s]
dt = 0.2                # timestep [s]
area = [50]             # airspace area
#marge = 0.1            # stop criteria for accuracy purposes
#Za = 1.96

for i in range(len(max_number)):
    for j in range(len(area)):
        #lists for collisions
#        LOS = []
#        averages = []
        n_runs = []
        
        #Looping
        looping = True
        trial = 0
#        f = open("results_" + str(max_number[i]) +"_"+ str(area[j]) + "_"+str(Sim_type)+".txt","w")

        while looping == True:
#            nr_LOS,nr_CLOS,nr_NMAC,nr_MAC,planes,angles = simrun(max_number[i],t_simulated,area[i],dt,Map)
            simrun(max_number[i],t_simulated,area[i],dt,Map)
#            LOS.append(nr_LOS)        
            
            trial = trial + 1
            print "run",trial," finished..."
            
            if trial == 1:   # this loop overwrites the next loop and is to make sure the simulator is only run once for testing purposes
                looping = False # when this loop is removed, the simulator only stops when the stop criteria is reached or forced to stop
            
#            if len(LOS)>=100: # This loop determines when the simulator should stop running. 
#                average1 = np.mean(LOS)
#                average2 = np.mean(eff_averages)
#                d1 = marge * average1
#                d2 = marge * average2
#                S1 = np.std(LOS)
#                S2 = np.std(eff_averages)
#                if 2 *Za * S1 /np.sqrt(trial) < d1 and 2 *Za * S2 /np.sqrt(trial) < d2:
#                    looping = False        
#        f.close()
