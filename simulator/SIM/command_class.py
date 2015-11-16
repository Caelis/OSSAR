# -*- coding: utf-8 -*-
"""
Created on Fri Nov 13 16:09:10 2015

@author: Leo

Package: main
Module: command_class
module dependence: atc_class, aircraft_class

description: create commands for each aircraft

Input:
Output:

"""
class command:
    def __init__(self, command_type, distance, value, commander, recipient, send_time, status):
        self.type = command_type
        self.distance = distance
        self.value = value
        self.commander = commander
        self.recipient = recipient
        self.send_time = send_time
        self.status = status #send: 1, received: 2, ackowledged: 4, denied: 8, executed: 16
