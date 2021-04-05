#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os
import math
import numpy as np
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QWidget
from pyqtlet import L, MapWidget

class Map(QWidget):
    def __init__(self,start_lat,start_lng):
        # Setting up the widgets and layout
        super().__init__()
        self.origin = [start_lat,start_lng]

        self.mapWidget = MapWidget()
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.mapWidget)
        self.setLayout(self.layout)
        self.curr_waypoint = self.origin

        # pyqlet maps
        self.map = L.map(self.mapWidget)
        self.map.setView(self.origin, 22)
        L.tileLayer('http://{s}.tile.osm.org/{z}/{x}/{y}.png').addTo(self.map)

        self.createOrigin(self.origin[0], self.origin[1])


    def createOrigin(self,x,y):
        self.spawn = L.marker([x,y,0])
        self.spawn.bindPopup("Ground Station")
        self.map.addLayer(self.spawn)
        # create icon
        loc = os.getcwd()
        loc_gs = "file://"+loc + "/ground_station.jpeg"
        self.map.runJavaScript(f'var iconlocation = "{loc_gs}"')
        self.map.runJavaScript('var gsIcon = L.icon({iconUrl: iconlocation});')
        self.map.runJavaScript(f'{self.spawn.jsName}.setIcon(gsIcon);')
        # create argos marker
        self.argos_pos = L.marker([x,y,0])
        self.argos_pos.bindPopup("ARGOS")
        self.map.addLayer(self.argos_pos)
        loc_argos = "file://"+loc + "/argos_icon.png"
        self.map.runJavaScript(f'var argosiconlocation = "{loc_argos}"')
        self.map.runJavaScript('var argosIcon = L.icon({iconUrl: argosiconlocation});')
        self.map.runJavaScript(f'{self.argos_pos.jsName}.setIcon(argosIcon);')
        # create position line
        self.path = [[x,y]]
        self.path_marker = L.polyline(self.path)
        self.map.addLayer(self.path_marker)



    def createMarker(self,x,y,name):
        m = L.marker([x,y])
        m.bindPopup(name)
        self.map.addLayer(m)
        self.curr_waypoint = [x,y]
        # create line to new waypoint
        self.map.runJavaScript(f'var oc = {[self.path[-1],[x,y]]}')
        self.map.runJavaScript('var p = L.polyline(oc,{color: "red"}).addTo(map)')

    def updatePath(self,x,y):
        # update path
        self.path.append([x,y])
        self.path_marker = L.polyline(self.path)
        self.map.addLayer(self.path_marker)


    def getDist(self):
        R = 637100
        dlon = self.curr_waypoint[1] - self.path[-1][1]
        dlat = self.curr_waypoint[0] - self.path[-1][0]
        a = math.sin(dlat / 2)**2 + math.cos(self.path[-1][0]) * math.cos(self.curr_waypoint[0]) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = abs(R * c)
        return distance