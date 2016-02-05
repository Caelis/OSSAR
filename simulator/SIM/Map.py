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

disp_link_density = True   # display density of each link
disp_aircaft_id = True      # display aircraft id number
disp_waypoint_id = True    # display waypoint id number
disp_links = True           # display the links between each waypoint
disp_radar_aircraft = False  # display which other aircraft an aircaft sees

def map_initialization(wp_database):
    pg.init()
    reso = (900,900)
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
        piclist.append(pg.transform.rotozoom(plane_pic,i,(1./14.)))
        rectlist.append(piclist[i].get_rect())
    return reso, scr, scrrect, plane_pic, piclist, X_waypoint, Y_waypoint
    
def map_running(reso,scr,scrrect,plane_pic,piclist,ATC_list,rectlist,running,r,X_waypoint,Y_waypoint,wp_database,wpl_database,graph):
    pg.draw.rect(scr,(255,255,255),scrrect)
    # draw links
    if disp_links:
        for i in xrange (len(wpl_database)):
            source = wpl_database[i][0]
            target = wpl_database[i][1]
            wp_map_x_1 = int((float(wp_database[source][1]) + r)/ (2*r) * reso[0])
            wp_map_y_1 = int((float(wp_database[source][2]) - r)/ (2 * r) *reso[1]) * -1
            wp_map_x_2 = int((float(wp_database[target][1]) + r)/ (2*r) * reso[0])
            wp_map_y_2 = int((float(wp_database[target][2]) - r)/ (2 * r) *reso[1]) * -1
            pg.draw.line(scr, (  0,   0, 255), (wp_map_x_1, wp_map_y_1), (wp_map_x_2, wp_map_y_2), 1)
    # draw link denstiy
    if disp_link_density:
        for key,value in graph.adjacency_iter():
            for inner_key,inner_value in value.items():
                density_string = str(inner_value['density'])
    
                center_x = wp_database[key][1] + (wp_database[inner_key][1]-wp_database[key][1])/2
                center_y = wp_database[key][2] + (wp_database[inner_key][2]-wp_database[key][2])/2
                center_x = int((float(center_x) + r)/ (2*r) * reso[0])
                center_y = int((float(center_y) - r)/ (2 * r) *reso[1]) * -1
    
                font = pg.font.Font(None, 14)
    
                if wp_database[key][2] == wp_database[inner_key][2]:
                    y_add = 10
                    x_add = 0
                else:
                    y_add = 0
                    x_add = 10
    
                if inner_key>key:
                    y_add = -y_add
                    x_add = -x_add
    
                text = font.render(density_string, 1, (10, 10, 10))
                textpos = text.get_rect()
                textpos.centerx = center_x + x_add
                textpos.centery = center_y + y_add
                scr.blit(text, textpos)
    # draw aircraft and aircraft id
    for atc in ATC_list:
        for plane in atc.locp:
            deg = int(degrees(plane.heading)-90)
            # convert from x/y coordinates to map coordinates
            plane_map_x = int((plane.x_pos + r)/ (2*r) * reso[0])
            plane_map_y = int((plane.y_pos - r)/ (2 * r) *reso[1]) * -1
            rectlist[deg].centerx = plane_map_x
            rectlist[deg].centery = plane_map_y
            scr.blit(piclist[deg],rectlist[deg])
            
            #diplay aircraft id
            if disp_aircaft_id:
                # id_string = str(plane.conflict)
                # id_string = str(round(plane.distance_to_atc,1))
                id_string = 'Going from ' + str(plane.atc[0]) + ' to ' + str(plane.atc[1]) + ' dist: ' + str(round(plane.distance_to_atc,1))
                font = pg.font.Font(None, 14)
                text = font.render(id_string, 1, (10, 10, 10))
                center_x_id = plane.x_pos + 75
                center_y_id = plane.y_pos + 75      
                center_x_id = int((float(center_x_id) + r)/ (2*r) * reso[0])
                center_y_id = int((float(center_y_id) - r)/ (2 * r) *reso[1]) * -1
                textpos = text.get_rect()
                textpos.centerx = center_x_id
                textpos.centery = center_y_id
                scr.blit(text, textpos)
            
            #siplay aircraft radar visibilty
            if disp_radar_aircraft:
                id_string_a = []
                for plane_radar in plane.radar:
                    id_string_a.append(plane_radar.id)
                id_string_a = str(id_string_a)
                id_string_b = id_string_a.translate(None,'[')
                id_string = id_string_b.translate(None,']')
                font = pg.font.Font(None, 14)
                text = font.render(id_string, 1, (10, 10, 10))
                center_x_id = plane.x_pos + 75
                center_y_id = plane.y_pos - 75      
                center_x_id = int((float(center_x_id) + r)/ (2*r) * reso[0])
                center_y_id = int((float(center_y_id) - r)/ (2 * r) *reso[1]) * -1
                textpos = text.get_rect()
                textpos.centerx = center_x_id
                textpos.centery = center_y_id
                scr.blit(text, textpos)                
            
    # draw waypoint names
    for i in xrange (len(wp_database)):
        wp_map_x = int((float(wp_database[i][1]) + r)/ (2*r) * reso[0])
        wp_map_y = int((float(wp_database[i][2]) - r)/ (2 * r) *reso[1]) * -1
        pg.draw.circle(scr, (255,0,0), (wp_map_x, wp_map_y), 3)
        if disp_waypoint_id:
            font = pg.font.Font(None, 14)
            text = font.render(str(i) + '/' + str(len(ATC_list[i].locp)), 1, (10, 10, 10))
            textpos = text.get_rect()
            textpos.centerx = wp_map_x+10
            textpos.centery = wp_map_y+10
            scr.blit(text, textpos)  

#        pg.draw.circle(scr, (255,0,0), (int((X_waypoint[i]+r) / (2*r) * reso[0]), int(((Y_waypoint[i]-r) / (2 * r) *reso[1]) * -1)), 3)
    pg.display.flip()  
    pg.event.pump() 
    keys = pg.key.get_pressed()

    if keys[pg.K_ESCAPE]:
        running = False
        print 'abort'
        
    return running