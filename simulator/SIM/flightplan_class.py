'''
Package: fLeet
Module: flightplan

description: This module creates flightplans based on begin and end coordinates, taking into account airspace rules and waypoints

Input:
Output:

Package dependence:
'''


#import python modules
from math import *
import numpy as np
import os

class Flightplan:
        
    def __init__(self):
       return

    # Creates a flightplan using the input of the planes
    def create_flightplan(self, id_nr, x_beg2, y_beg2, x_des2, y_des2, data3,r):      
        flightplan = []
#        leg = 0
        if x_beg2 <= -0.5*np.sqrt(2*(r**2)) and not x_des2 <= -0.5*np.sqrt(2*(r**2)):
            if y_des2 <= -0.5*np.sqrt(2*(r**2)):
                wp_nr = 7
            elif x_des2 >= 0.5*np.sqrt(2*(r**2)):
                wp_nr = 11
            elif y_des2 >= 0.5*np.sqrt(2*(r**2)):
                wp_nr = 0
            flightplan.append([int(data3[wp_nr][1]),int(data3[wp_nr][2])])
        elif y_beg2 <= -0.5*np.sqrt(2*(r**2)) and not y_des2 <= -0.5*np.sqrt(2*(r**2)):
            if x_des2 <= -0.5*np.sqrt(2*(r**2)):
                wp_nr = 6
            elif x_des2 >= 0.5*np.sqrt(2*(r**2)):
                wp_nr = 5
            elif y_des2 >= 0.5*np.sqrt(2*(r**2)):
                wp_nr = 10
            flightplan.append([int(data3[wp_nr][1]),int(data3[wp_nr][2])])
        elif x_beg2 >= 0.5*np.sqrt(2*(r**2)) and not x_des2 >= 0.5*np.sqrt(2*(r**2)):
            if x_des2 <= -0.5*np.sqrt(2*(r**2)):
                wp_nr = 9
            elif y_des2 <= -0.5*np.sqrt(2*(r**2)):
                wp_nr = 4
            elif y_des2 >= 0.5*np.sqrt(2*(r**2)):
                wp_nr = 3
            flightplan.append([int(data3[wp_nr][1]),int(data3[wp_nr][2])])
        elif y_beg2 >= 0.5*np.sqrt(2*(r**2)) and not y_des2 >= 0.5*np.sqrt(2*(r**2)):
            if x_des2 <= -0.5*np.sqrt(2*(r**2)):
                wp_nr = 1
            elif y_des2 <= -0.5*np.sqrt(2*(r**2)):
                wp_nr = 8
            elif x_des2 >= 0.5*np.sqrt(2*(r**2)):
                wp_nr = 2
            flightplan.append([int(data3[wp_nr][1]),int(data3[wp_nr][2])])
        flightplan.append([x_des2, y_des2])

#        for i in xrange(len(flightplan)-1):
#            legi = sqrt((flightplan[i+1][0]-flightplan[i][0])**2+(flightplan[i+1][1]-flightplan[i][1])**2)            
#            leg = leg + legi
#        leg = leg + sqrt((flightplan[0][0]-x_beg2)**2+(flightplan[0][1]-y_beg2)**2)
#        comb = sqrt((flightplan[-1][0]-x_beg2)**2+(flightplan[-1][1]-y_beg2)**2)
#        if int(leg) == 0:
#            eff = 0
#        else:
#            eff = comb / leg
        return flightplan
