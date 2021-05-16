
import argparse
import base64
from datetime import datetime
import os
import shutil

import numpy as np
import socketio
import eventlet
import eventlet.wsgi
from PIL import Image
from flask import Flask
from io import BytesIO
import cv2
# from keras.models import load_model

import utils
import _thread
import time
from typing import Dict, List, Iterator, Tuple, TypeVar
import numpy as np
# import Astar
sio = socketio.Server()
app = Flask(__name__)
model = None
prev_image_array = None

MAX_SPEED = 25
MIN_SPEED = 10

speed_limit = MAX_SPEED

IMAGE_H = 66
IMAGE_W = 200

src = np.float32([[0, IMAGE_H], [200, IMAGE_H], [0, 0], [IMAGE_W, 0]])
dst = np.float32([[80, IMAGE_H], [120, IMAGE_H], [0, 0], [IMAGE_W, 0]])
M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation

map_png = cv2.imread("./map_png.png")
Lon_Or = 106.65644
Lat_Or = 10.77654

LatOrigin = 10.773390000000006
LonOrigin = 106.65971499999999

global_path = []
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
GridLocation = Tuple[int, int]

Location = TypeVar('Location')
class SquareGrid:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.walls: List[GridLocation] = []
        self.weights: Dict[GridLocation, float] = {}
        
    def in_bounds(self, id: GridLocation) -> bool:
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def passable(self, id: GridLocation) -> bool:
        return id not in self.walls
    
    def neighbors(self, id: GridLocation) -> Iterator[GridLocation]:
        (x, y) = id
        neighbors = [(x+1, y), (x-1, y), (x, y-1), (x, y+1),(x+1, y+1), (x-1, y-1), (x+1, y-1), (x-1, y+1)] # E W N S
        # see "Ugly paths" section for an explanation:
        if (x + y) % 2 == 0: neighbors.reverse() # S N W E
        results = filter(self.in_bounds, neighbors)
        results = filter(self.passable, results)
        return results
    def cost(self, from_node: GridLocation, to_node: GridLocation) -> float:
        if(self.weights.get(to_node, 1) != None):
            return self.weights.get(to_node, 1)
        return 1
diagram4 = SquareGrid(10, 10)

# utility functions for dealing with square grids
def from_id_width(id, width):
    return (id % width, id // width)

def draw_tile(graph, id, style):
    r = " . "
    if 'number' in style and id in style['number']: r = " %-2d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = " > "
        if x2 == x1 - 1: r = " < "
        if y2 == y1 + 1: r = " v "
        if y2 == y1 - 1: r = " ^ "
    if 'path' in style and id in style['path']:   r = " @ "
    if 'start' in style and id == style['start']: r = " A "
    if 'goal' in style and id == style['goal']:   r = " Z "
    if id in graph.walls: r = "###"
    return r

def draw_grid(graph, **style):
    print("___" * graph.width)
    for y in range(graph.height):
        for x in range(graph.width):
            print("%s" % draw_tile(graph, (x, y), style), end="")
        print()
    print("~~~" * graph.width)

import heapq
import collections

T = TypeVar('T')
class Queue:
    def __init__(self):
        self.elements = collections.deque()
    
    def empty(self) -> bool:
        return not self.elements
    
    def put(self, x: T):
        self.elements.append(x)
    
    def get(self) -> T:
        return self.elements.popleft()

class PriorityQueue:
    def __init__(self):
        self.elements: List[Tuple[float, T]] = []
    
    def empty(self) -> bool:
        return not self.elements
    
    def put(self, item: T, priority: float):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self) -> T:
        return heapq.heappop(self.elements)[1]
def reconstruct_path(came_from: Dict[Location, Location],
                     start: Location, goal: Location) -> List[Location]:
    current: Location = goal
    path: List[Location] = []
   
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional
    return path



def heuristic(a: GridLocation, b: GridLocation) -> float:
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

def a_star_search(graph: SquareGrid, start: Location, goal: Location):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from: Dict[Location, Optional[Location]] = {}
    cost_so_far: Dict[Location, float] = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current: Location = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(next, goal)
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far
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
def pix2global(x,y,size_x,size_y):
    lat_x = x*(106.66226 - 106.65644)/size_x + 106.65644
    lat_y = y*(10.77654 - 10.77023)/size_y + 10.77023
    return lat_x, lat_y
def global2pix(Lat,Lon,Lat_Or,Lon_Or):
    x, y = int((Lon - Lon_Or + 100)*map_png.shape[1]/(106.66226 - 106.65644)),int((Lat_Or - Lat)*map_png.shape[0]/(10.77654 - 10.77023))
    return x,y
result = np.zeros((66,200))

def draw_map( threadName,pitch, Lat, Lon):
    global map_png, result, global_path
    
    demo = np.copy(map_png)
    x, y = int((Lon - Lon_Or + 100)*map_png.shape[1]/(106.66226 - 106.65644)),int((Lat_Or - Lat)*map_png.shape[0]/(10.77654 - 10.77023))

    # map_png = cv2.circle(map_png, ( int((Lon - Lon_Or + 100)*map_png.shape[1]/(106.66226 - 106.65644)),int((Lat_Or - Lat)*map_png.shape[0]/(10.77654 - 10.77023)) ), radius=1, color=(0, 0, 255), thickness=-1)
    p1,p2,p3,p4 = get_map_pos(pitch, x, y)
    # p5, r_p5 = get_map_loc(pitch, x, y, 3, 14)
    # print(r_p5[0] - x, r_p5[1] - y)
    demo = cv2.circle(demo,p1, radius=1, color=(0, 0, 255), thickness=-1)
    demo = cv2.circle(demo,p2, radius=1, color=(0, 0, 255), thickness=-1)
    demo = cv2.circle(demo,(x,y), radius=1, color=(0, 0, 255), thickness=-1)
    cv2.line(demo, (x,y), (100,100), (0, 255, 0), thickness=1)
    demo = cv2.circle(demo,p3, radius=1, color=(0, 0, 255), thickness=-1)
    demo = cv2.circle(demo,p4, radius=1, color=(0, 0, 255), thickness=-1)
    # demo = cv2.circle(demo,p5, radius=1, color=(0, 0, 255), thickness=-1)
    # k,l = 0,0
    # for i in range(p1[0],p4[0],1):
    #     for j in range(p1[1],p4[1],1):
    #         k += 1
    #         l += 1

            # demo[i][j] = result[k][l]
    resized = cv2.resize(demo[(y - 100):(y+100),(x-100):(x+100)], (400,400), interpolation = cv2.INTER_AREA)


    cv2.imshow("map1",resized)

    cv2.imshow("map",demo)
    
    # print("  Distance: ",  ConvertGPStoUCS(Lat_Or,Lon_Or,Lat, Lon+100) )
    

    cv2.waitKey(0)
def PID(err,sp):
    Kp = 0.1
    if(err > 180):
        err = -180 + err 
    angle = Kp*(sp - err)
    # send_control(angle/180*np.pi, 50)ư
    return angle 

@sio.on('telemetry')
def telemetry(sid, data):
    global image
    if data:

        # The current steering angle of the car
        steering_angle = float(data["steering_angle"])
        # The current throttle of the car
        throttle = float(data["throttle"])
        # The current speed of the car
        speed = float(data["speed"])

        Lat = float(data["Lat"])

        Lon = float(data["Lon"])

        roll = float(data["r"])

        pitch = float(data["p"])

        yaw = float(data["y"])
        # The current image from the center camera of the car
        image = Image.open(BytesIO(base64.b64decode(data["image"])))
        # save frame
        # if args.image_folder != '':
        #     timestamp = datetime.utcnow().strftime('%Y_%m_%d_%H_%M_%S_%f')[:-3]
        #     image_filename = os.path.join(args.image_folder, timestamp)
        #     image.save('{}.jpg'.format(image_filename))
            
        try:
            image = np.asarray(image)       # from PIL image to numpy array
            
            image = utils.preprocess(image) # apply the preprocessing
            
            
            #waits for user to press any key  
            #(this is necessary to avoid Python kernel form crashing) 
            
            _thread.start_new_thread( print_time, ("Thread-1", image,Lat,Lon,pitch ) )
            _thread.start_new_thread( draw_map, ("Thread-1",pitch, Lat,Lon ) )
            image = np.array([image])       # the model expects 4D array
            
            # # predict the steering angle for the image
            # steering_angle = float(model.predict(image, batch_size=1))
            # # lower the throttle as the speed increases
            # # if the speed is above the current speed limit, we are on a downhill.
            # # make sure we slow down first and then go back to the original max speed.
            # global speed_limit
            # if speed > speed_limit:
            #     speed_limit = MIN_SPEED  # slow down
            # else:
            #     speed_limit = MAX_SPEED
            #send_control(steering_angle, throttle)
            # throttle = 1.0 - steering_angle**2 - (speed/speed_limit)**2
            PID(pitch,0)
            # print('{} {} {} {} {} {} {} {}'.format(steering_angle, speed, throttle,roll,pitch,yaw, Lat, Lon + 100))
            sio.emit('manual', data={}, skip_sid=True)
            print()
        except Exception as e:
            print(e)
        
    else:
        # NOTE: DON'T EDIT THIS.
        sio.emit('manual', data={}, skip_sid=True)


@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)
    send_control(0, 0)


def send_control(steering_angle, throttle):
    sio.emit(
        "steer",
        data={
            'steering_angle': steering_angle.__str__(),
            'throttle': throttle.__str__()
        },
        skip_sid=True)


if __name__ == '__main__':
    # parser = argparse.ArgumentParser(description='Remote Driving')
    # parser.add_argument(
    #     'model',
    #     type=str,
    #     help='Path to model h5 file. Model should be on the same path.'
    # )
    # parser.add_argument(
    #     'image_folder',
    #     type=str,
    #     nargs='?',
    #     default='',
    #     help='Path to image folder. This is where the images from the run will be saved.'
    # )
    # args = parser.parse_args()

    # # model = load_model(args.model)

    # if args.image_folder != '':
    #     print("Creating image folder at {}".format(args.image_folder))
    #     if not os.path.exists(args.image_folder):
    #         os.makedirs(args.image_folder)
    #     else:
    #         shutil.rmtree(args.image_folder)
    #         os.makedirs(args.image_folder)
    #     print("RECORDING THIS RUN ...")
    # else:
    #     print("NOT RECORDING THIS RUN ...")

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)
    
    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
    send_control(0, 0)