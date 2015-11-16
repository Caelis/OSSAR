# -*- coding: utf-8 -*-
"""
Created on Mon Nov 09 13:08:13 2015
@author: Leo

Package: Import
module: main
module dependence: Fleet

description:
Eleborate description: (link to formulas)

Input: All documents which need to be imported
Output:
"""

#import modules
import os

#importing the aircraft database
text = open(os.getcwd()+"/SIM/aircraft_database.txt","r")
lines = text.readlines()[1:]
data = [lines.split() for lines in lines]
text.close()

#import waypoint database
wp_database_text = open(os.getcwd()+"/SIM/waypoint_database.txt","r")
wp_database_lines = wp_database_text.readlines()[1:]
wp_database = [wp_database_lines.split() for wp_database_lines in wp_database_lines]
wp_database_text.close()

#import waypoint link database
wpl_database_text = open(os.getcwd()+"/SIM/waypoint_links.txt","r")
wpl_database_lines = wpl_database_text.readlines()[1:]
wpl_database = [wpl_database_lines.split() for wpl_database_lines in wpl_database_lines]
wpl_database_text.close()

#import arrival database
ar_database_text = open(os.getcwd()+"/SIM/flightdata.txt","r")
ar_database_lines = ar_database_text.readlines()[1:]
ar_database = [ar_database_lines.split() for ar_database_lines in ar_database_lines]
ar_database_text.close()

#make gate database
waypoint_gate = [elem for elem in wp_database if int(elem[3]) == 1]
g_database = [elem[0] for elem in waypoint_gate]

#make runway database
waypoint_runway = [elem for elem in wp_database if int(elem[3]) == 4]
rw_database = [elem[0] for elem in waypoint_runway]