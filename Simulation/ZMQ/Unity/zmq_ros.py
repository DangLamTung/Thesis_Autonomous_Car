#! /usr/local/bin/python3
import zmq
import time
import random

import base64
import numpy as np
import cv2
from PIL import Image
from io import BytesIO
import _thread
from drive import *
from stanley_controller import *
import json


context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:12346")

TIMEOUT = 10000
def make_matrix(roll,pitch,heading,x0,y0,z0):
    a = np.radians(roll)
    b = np.radians(pitch)
    g = np.radians(heading)

    T = np.array([[ np.cos(b)*np.cos(g), (np.sin(a)*np.sin(b)*np.cos(g) + np.cos(a)*np.sin(g)), (np.sin(a)*np.sin(g) - np.cos(a)*np.sin(b)*np.cos(g)), x0],
    [-np.cos(b)*np.sin(g), (np.cos(a)*np.cos(g) - np.sin(a)*np.sin(b)*np.sin(g)), (np.sin(a)*np.cos(g) + np.cos(a)*np.sin(b)*np.sin(g)), y0],
    [ np.sin(b), -np.sin(a)*np.cos(b), np.cos(a)*np.cos(b), z0],
    [ 0, 0, 0, 1]])
    return T
def Rz(theta):
    return np.matrix([[ np.cos(theta), -np.sin(theta)],
                   [ np.sin(theta), np.cos(theta) ],
                  ])
def draw_map( image,pitch, Lat, Lon,global_path):
    global map_png, result
    
    demo = np.copy(map_png)
    x, y = int((Lon + 100 - Lon_Or)*map_png.shape[1]/(106.66226 - 106.65644)),int((Lat_Or - Lat)*map_png.shape[0]/(10.77654 - 10.77023))

    # map_png = cv2.circle(map_png, ( int((Lon - Lon_Or + 100)*map_png.shape[1]/(106.66226 - 106.65644)),int((Lat_Or - Lat)*map_png.shape[0]/(10.77654 - 10.77023)) ), radius=1, color=(0, 0, 255), thickness=-1)
    p1,p2,p3,p4 = get_map_pos(pitch, x, y)
    # p5, r_p5 = get_map_loc(pitch, x, y, 3, 14)
    # print(r_p5[0] - x, r_p5[1] - y)
    demo = cv2.circle(demo,p1, radius=1, color=(0, 0, 255), thickness=-1)
    demo = cv2.circle(demo,p2, radius=1, color=(0, 0, 255), thickness=-1)
    demo = cv2.circle(demo,(x,y), radius=1, color=(0, 0, 255), thickness=-1)
    # cv2.line(demo, (x,y), (100,100), (0, 255, 0), thickness=1)
    demo = cv2.circle(demo,p3, radius=1, color=(0, 0, 255), thickness=-1)
    demo = cv2.circle(demo,p4, radius=1, color=(0, 0, 255), thickness=-1)
    map_path_x = []
    map_path_y = []
    global_path_distance = []
    global_path_inframe = []
    # for p in global_path:
    #     # print(p)
    #     dis_to_point = ConvertGPStoUCS(Lat,Lon+100,p[0], p[1])
    #     global_path_distance.append(dis_to_point)
    #     (x_,y_) = global2pix(p[0],p[1]-100,Lat_Or,Lon_Or)

    #     if (dis_to_point[0] > -3 and dis_to_point[0] < 3):
    #         if (dis_to_point[1] > 0 and dis_to_point[0] < 3):
    #             global_path_inframe.append(dis_to_point)
    #     # print(x,y)
    #     map_path_x.append(x_)
    #     map_path_y.append(y_)
    #     demo = cv2.circle(demo,(x_,y_), radius=1, color=(0, 0, 255), thickness=-1)
    # print(global_path_distance)
    # state = State(x=x, y=y, yaw = pitch, v=0.0)
    # # # print(x,y)
    # target_idx, _ = calc_target_index(state, map_path_x, map_path_y)
    # if(target_idx < len(map_path_x)):
    #     global_angle = np.arctan((global_path[target_idx][0] - global_path[target_idx + 1][0])/(global_path[target_idx][1] - global_path[target_idx + 1][1] ))
    # print(target_idx)
    # print(global_angle*180/np.pi)
    # print(pitch - global_angle*180/np.pi)
    # demo = cv2.circle(demo,(map_path_x[target_idx],map_path_y[target_idx]), radius=3, color=(0, 255, 0), thickness=-1)
    # # demo = cv2.circle(demo,p5, radius=1, color=(0, 0, 255), thickness=-1)
    # # k,l = 0,0
    # # for i in range(p1[0],p4[0],1):
    # #     for j in range(p1[1],p4[1],1):
    # #         k += 1
    # #         l += 1

    # #         # demo[i][j] = result[k][l]
    resized = cv2.resize(demo[(y - 100):(y+100),(x-100):(x+100)], (400,400), interpolation = cv2.INTER_AREA)

    


    cv2.imshow("map1",resized)
    cv2.imshow("map",demo)
    
    # print("  Distance: ",  ConvertGPStoUCS(Lat_Or,Lon_Or,Lat, Lon+100) )

    # The black region in the mask has the value of 0,
    # so when multiplied with original image removes all non-blue regions
    
    # cv2.imshow("map_o",occupancy_map)
    # print(occupancy_map)
    # time.sleep(0.5)

    # cv2.imshow("im",image)

    # # warped_img = cv2.warpPerspective(image, M, (IMAGE_W, IMAGE_H)) # Image warping
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # # Threshold of blue in HSV space

    lower_blue = np.array([0, 0, 0])
    upper_blue = np.array([100, 100, 100])
 
    # preparing the mask to overlay
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    result_mask = cv2.bitwise_and(image, image, mask = mask)   
  
    # # print(x,y)
    
    # #predict the position
    # x1, y1 = 0, 0
    # x2, y2 = 200, 400
    # line_thickness = 1


    # occupancy_map = np.zeros((16,32))
    # dis_map = np.ones((16,32))*1000
    # # p1,p2,p3,p4 = get_map_pos(pitch, x, y)
    
    
   
    # lon_now, lat_now = pix2global(x,y,map_png.shape[1],map_png.shape[0])


    # # print("  Distance: ",  ConvertGPStoUCS(lat_goal,lon_goal,lat_now, lon_now) )
    # for i in range(16):
    #     for j in range(32):
            
    #         if( (mask[i*10:(i*10+ 10),j*10:(j*10 + 10)] == 255).sum( ) > 80):
    #             occupancy_map[i][j] = 1
              

    #         cv2.line(result, (0, (i+1)*10), (320, (i+1)*10), (255, 255,255), thickness=line_thickness)
    #         cv2.line(result, (j*10, 0), (j*10, 160), (255, 255,255), thickness=line_thickness)
          
    # road = np.flip( occupancy_map, 0)
    # road = np.flip( occupancy_map, 1)
    # g = SquareGrid(32,16)
    # # mapping occupancy map to real map
    
    # map_pos = []
    # wall = []
    # for i in range(16):
    #     for j in range(32):
    #         if(road[i][j] == 0):
    #             wall.append((j,i))
    #         else:
    #             _, r_p5 = get_map_loc(pitch, x, y, i, j)
    #             map_pos.append(r_p5)
    #             dis = np.hypot(map_path_x[target_idx] - r_p5[0],map_path_y[target_idx] - r_p5[1] )
    #             lon_now, lat_now = pix2global(r_p5[0],r_p5[1],map_png.shape[1],map_png.shape[0])
    #             dis_map[i][j] = dis
    # # print(dis_map)s
    # min_node = np.argmin(dis_map,axis = 1)
    
    # #map occupancy to real map
    # rot_map = Rz(pitch)

    # # target_to_local = np.dot(rot_map,np.array( [map_path_x[target_idx],map_path_y[target_idx],0,1]))
    # # print(np.where(np.logical_and(map_path_x>=p1[0], map_path_x<=p3[0])))
    # for global_in in global_path_inframe:
     
    #     cv2.circle(result_mask,(int((global_in[0] + 3)/3*result_mask.shape[0]), int(global_in[1]/3*result_mask.shape[1])), radius=1, color=(0, 0, 255), thickness=-1)
                
    
    # # cv2.circle(result_mask,(map_path_x[target_idx],map_path_y[target_idx]), radius=3, color=(0, 255, 0), thickness=-1)
    # goal_x, goal_y = np.unravel_index(dis_map.argmin(), dis_map.shape)
    # start, goal = (16, 15),(min_node[0],0)
    # # print(goal)
    # for i in range(16):
    #     index_pos_list = [ j for j in range(len(occupancy_map[:,j])) if occupancy_map[i][j] == 1 ]
    #     print(index_pos_list)   
    #     if(len(index_pos_list) > 2):    
    #         point = (int( (index_pos_list[len(index_pos_list)-1]- index_pos_list[0] )*10+5),i*10+5)
            
    #         cv2.circle(result_mask, point, radius=1, color=(0, 0, 255), thickness=-1)
    # # # print(road)n
    # # g.walls = wall
    # # # print(diagram4.weights)
    # # # print(diagram4.cost(start,goal))
    # # found_way = 0
    # # came_from, cost_so_far = a_star_search(g, start, goal)
    # # # print(came_from)
    # # for loc, op in came_from.items():


    # #     path = reconstruct_path(came_from, start=start, goal=goal)
        
    # #     for i in range(len(path)):
    # #         if(i>1):
    # #             point1 = ((path[i-1][0])*10+5,path[i-1][1]*10+5)
    # #             point2 = ((path[i][0])*10+5,path[i][1]*10+5)
    # #             cv2.line(result_mask, point1, point2, (0, 255, 0), thickness=line_thickness)
    # #         road[path[i][1]][path[i][0]] = 2
    # # print(road)
    # # _thread.start_new_thread( draw_grid, ( g ) )

    # # print(came_from)
    # # draw_grid(g, path=reconstruct_path(came_from, start=start, goal=goal))    

    # # result = cv2.warpPerspective(result, M, (IMAGE_W, IMAGE_H)) # Image warping
    # # time.sleep(100)
    cv2.imshow('frame', result_mask)
    # cv2.imshow("im1",image)
    cv2.waitKey(0)

def send_control(raw,angle,velocity):
    raw.write(b's')
    raw.write(bytes([int(angle / 10)]))
    raw.write(bytes([angle % 10]))
    raw.write(bytes([velocity]))
    raw.write(bytes([(velocity + angle) % 37]))
    raw.write(b'e')    
import threading
import serial
import pynmea2
import io

connected = False
# raw.open()
class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)
 
def zmq_connect(global_path):
    global socket
    while True:
        socket.send_string(str(0) + ","+str(1))
        poller = zmq.Poller()
        poller.register(socket, zmq.POLLIN)
        evt = dict(poller.poll(TIMEOUT))
        if evt:
            if evt.get(socket) == zmq.POLLIN:
                response = socket.recv(zmq.NOBLOCK)
                arr = response.split()	
                image = Image.open(BytesIO(base64.b64decode(arr[0])))
                image = np.asarray(image)   

                # bird_image = Image.open(BytesIO(base64.b64decode(arr[6])))
                # bird_image = np.asarray(bird_image)   
                cv2.imshow("im",image)

                _thread.start_new_thread( draw_map, (image,float(arr[4])*180/np.pi, float(arr[1]), float(arr[2]),global_path) )
                #_thread.start_new_thread( draw_map, ("Thread-2", image,float(arr[1]), float(arr[2]),float(arr[4])*180/np.pi,global_path) )
                print(arr[1], arr[2], arr[3], arr[4], arr[5])
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                # print(response)
                
        time.sleep(0.1)
        socket.close()

        socket = context.socket(zmq.REQ)
        socket.connect("tcp://localhost:12346")
zmq_connect([100,100])
# while True:
#     socket.send_string("request")
#     poller = zmq.Poller()
#     poller.register(socket, zmq.POLLIN)
#     evt = dict(poller.poll(TIMEOUT))
#     if evt:
#         if evt.get(socket) == zmq.POLLIN:
#             response = socket.recv(zmq.NOBLOCK)
#             arr = response.split()	
#             image = Image.open(BytesIO(base64.b64decode(arr[0])))
#             image = np.asarray(image)   
#             # cv2.imshow("im",image)

#             _thread.start_new_thread( draw_map, ("Thread-1",float(arr[4]), float(arr[1]), float(arr[2]) ) )

#             print(arr[1], arr[2], arr[3])
#             # cv2.waitKey(0)
#            # print(response)
#             continue
#     time.sleep(0.5)
#     socket.close()
#     socket = context.socket(zmq.REQ)
#     socket.connect("tcp://localhost:12346")
