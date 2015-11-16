'''

Package: map
Module: map

description: The Map package provides a graphical representation of the simulator

Input:
Output:
'''
import pygame as pg
from math import *

#initializing values
piclist = []
rectlist = []

def map_initialization(wp_database):
    pg.init()
    reso = (600,600)
    scr = pg.display.set_mode(reso)
    scrrect = scr.get_rect()
    scr.fill((2,255,70))
    plane_pic= pg.image.load("blue-plane-hi.png")
    # set waypoints
    X_waypoint = []
    Y_waypoint = []

    for i in xrange (len(wp_database)):
        X_waypoint.append(int(wp_database[i][1]))
        Y_waypoint.append(int(wp_database[i][2]))    

    for i in xrange(0,360):
        piclist.append(pg.transform.rotozoom(plane_pic,i,(1./12.)))
        rectlist.append(piclist[i].get_rect())
    return reso, scr, scrrect, plane_pic, piclist, X_waypoint, Y_waypoint
    
def map_running(reso,scr,scrrect,plane_pic,piclist,ATC_list,rectlist,running,r,X_waypoint,Y_waypoint,wp_database):
    pg.draw.rect(scr,(255,255,255),scrrect)
#    pg.draw.circle(scr,(255,0,0),(reso[0]/2,reso[1]/2),reso[0]/2, 3)
    for atc in ATC_list:
        for plane in atc.locp:
            deg = int(degrees(plane.heading)-90)
            # convert from x/y coordinates to map coordinates
            plane_map_x = int((plane.x_pos + r)/ (2*r) * reso[0])
            plane_map_y = int((plane.y_pos - r)/ (2 * r) *reso[1]) * -1
            rectlist[deg].centerx = plane_map_x
            rectlist[deg].centery = plane_map_y
            scr.blit(piclist[deg],rectlist[deg])
        
    for i in xrange (len(wp_database)):
        wp_map_x = int((float(wp_database[i][1]) + r)/ (2*r) * reso[0])
        wp_map_y = int((float(wp_database[i][2]) - r)/ (2 * r) *reso[1]) * -1
        pg.draw.circle(scr, (255,0,0), (wp_map_x, wp_map_y), 3)
#        pg.draw.circle(scr, (255,0,0), (int((X_waypoint[i]+r) / (2*r) * reso[0]), int(((Y_waypoint[i]-r) / (2 * r) *reso[1]) * -1)), 3)
    pg.display.flip()  
    pg.event.pump() 
    keys = pg.key.get_pressed()

    if keys[pg.K_ESCAPE]:
        running = False
        print 'abort'
        
    return running