import argparse
import base64
from datetime import datetime
import os
import shutil

import numpy as np

from PIL import Image

from io import BytesIO

from pyroutelib3 import Router, Datastore 


Lon_Or = 106.65644
Lat_Or = 10.77654

LatOrigin = 10.773390000000006
LonOrigin = 106.65971499999999

map_data_type = {
  "weights": {
        "primary": 0.05, "secondary": 0.15, "tertiary": 0.3, "unclassified": 1,
        "residential": 1, "track": 1.5, "service": 1, "bridleway": 5, "path": 1.5
  },
  "name": "horse",
  "access": ["access", "horse","car"]
}
def FindMetersPerLat(lat): # Compute lengths of degrees
	
	m1 = 111132.92
	m2 = -559.82
	m3 = 1.175
	m4 = -0.0023
	p1 = 111412.84
	p2 = -93.5
	p3 = 0.118

	lat = np.deg2rad(lat)
	# Calculate the length of a degree of latitude and longitude in meters
	metersPerLat = m1 + (m2 * np.cos(2 * lat)) + (m3 * np.cos(4 * lat)) + (m4 * np.cos(6 * lat))
	metersPerLon = (p1 * np.cos(lat)) + (p2 * np.cos(3 * lat)) + (p3 * np.cos(5 * lat))
	return metersPerLat,metersPerLon
def ConvertGPStoUCS(_LatOrigin,_LonOrigin,Lat, Lon):
	metersPerLat,metersPerLon =  FindMetersPerLat(_LatOrigin)
	yPosition  = metersPerLat * (Lat - _LatOrigin)
	xPosition  = metersPerLon * (Lon - _LonOrigin)
	return xPosition, yPosition

def ConvertUCStoGPS(LatOrigin,LonOrigin,x, y):
	metersPerLat,metersPerLon =  FindMetersPerLat(LatOrigin)
	Lat_dis = ((LatOrigin + (y) / metersPerLat))
	Lon_dis = ((LonOrigin + (x) / metersPerLon))
	return Lat_dis,Lon_dis

def get_map_loc(pitch, x, y, grid_x, grid_y):
    pitch = pitch/180*np.pi
    if(grid_y > 10):
        loc_x = 10*(grid_x/6.0)*np.cos(pitch) + x - 10*(grid_y/10.0 - 1)*np.sin(-pitch)
        loc_y = 10*(grid_x/6.0)*np.sin(pitch) + y - 10*(grid_y/10.0 - 1)*np.cos(-pitch)
    else:
        loc_x = -10*(grid_x/6.0)*np.cos(pitch) + x - 10*(grid_y/10.0)*np.sin(-pitch)
        loc_y = -10*(grid_x/6.0)*np.sin(pitch) + y - 10*(grid_y/10.0)*np.cos(-pitch)
    
    return (int(loc_x),int(loc_y)), (loc_x,loc_y)

def get_map_pos(pitch, x, y):
    pitch = pitch/180*np.pi
    b_dl_x = 10*np.cos(pitch) + x
    b_dl_y = 10*np.sin(pitch) + y

    b_dr_x = -10*np.cos(pitch) + x
    b_dr_y = -10*np.sin(pitch) + y

    b_ul_x = 10*np.cos(pitch) + x - 10*np.sin(-pitch)
    b_ul_y = 10*np.sin(pitch) + y - 10*np.cos(-pitch)

    b_ur_x = -10*np.cos(pitch) + x - 10*np.sin(-pitch)
    b_ur_y = -10*np.sin(pitch) + y - 10*np.cos(-pitch)
    return (int(b_dl_x) ,int(b_dl_y)), (int(b_dr_x) ,int(b_dr_y)), (int(b_ul_x) ,int(b_ul_y)),(int(b_ur_x),int(b_ur_y))


def pix2global_gui(x,y,size_x = 680,size_y = 751):
    """
    convert pixel position on map_png to lat lon
    """

    lon = x*(106.66226 - 106.65644)/size_x + 106.65644
    lat = 10.77654 - y*(10.77654 - 10.77023)/size_y
    return lat, lon


def global2pix_gui(Lat,Lon,Lat_Or = LatOrigin,Lon_Or = LonOrigin ):
    """
    convert lat lon to pixel position on map_png
    """

    y, x = int((Lon- 106.65644)*680/(106.66226 - 106.65644)),int((10.77654 - Lat)*751/(10.77654 - 10.77023))
    return x,y

def convertPath(path):
    pathOnMap = []
    for p in path:
        pathOnMap.append(global2pix_gui(p[0],p[1]))
    return pathOnMap
def findRoute(waypoints):
    path = []

    router = Router("car","map.osm") # Initialise it
    
    if(len(waypoints) == 2):
        start = router.findNode(waypoints[0][0], waypoints[0][1]) # Find start and end nodes
        end = router.findNode(waypoints[1][0], waypoints[1][1])
        status, route = router.doRoute(start,end) # Find the route - a list of OSM nodes
        if status == 'success':
            routeLatLons = list(map(router.nodeLatLon, route)) # Get actual route coordinates
            for coord in routeLatLons:
                print(coord)
                path.append(coord)

            # print(routeLatLons)
    return path
from decimal import Decimal


def lin_equ(l1, l2):
    """Line encoded as l=(x,y)."""
    if(l2[0] != l1[0]):
        m = (l2[1] - l1[1]) / (l2[0] - l1[0])
        c = (l2[1] - (m * l2[0]))
        direction = 0
    else:
        m = (l2[0] - l1[0]) / (l2[1] - l1[1])
        c = (l2[0] - (m * l2[1]))
        direction = 1
    return m, c, direction
import math
#global path resolution
res = 0.5 # meter
def enhanceWaypoint(waypoints):
    path = []

    for i,coord in enumerate(waypoints[0:len(waypoints) - 1]):
        a,b, direction = lin_equ(waypoints[i+1], coord)
        distance = ConvertGPStoUCS(waypoints[i+1][1],waypoints[i+1][0],coord[1],coord[0])
        way_num = int(math.sqrt(distance[0]**2 + distance[1]**2)/res)
        for j in range(way_num):
            if(direction == 0):
                lat_res = (waypoints[i+1][0] - coord[0])/way_num
                lat = coord[0] + j*lat_res
                path.append((lat, a*lat+b))
            else:
                lon_res = (waypoints[i+1][1] - coord[1])/way_num
                lon = coord[1] + j*lon_res
                path.append((a*lon+b,lon))
            # print(routeLatLons)
    return path