#!/usr/bin/env python2
# -*- coding: utf-8 -*-
#============================================================================================
'''  Import libraries  '''
import requests
import polyline
import utm

import rospy
from sensor_msgs.msg import NavSatFix
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import tf as ros_tf

import numpy as np
#============================================================================================



#============================================================================================
'''   Init ROS node, input and output topics   '''
rospy.init_node("Google_Map_Global_Planner")
tf_listener = ros_tf.TransformListener()    

move_base_stt_input = "move_base/status"
gps_position_input  = "gps/filtered"
geo_goal_input      = "move_base/geo_goal"
map_goal_input      = "move_base/goal"

global_plan_output  = "move_base/GoogleMapGlobalPlanner/plan"
#============================================================================================



#============================================================================================
'''   This function converts a point in map coordinate into lat_long   '''
def map2ll(x_map,y_map):
    
    ros_tf_transformer = ros_tf.TransformerROS()
    try:
        tf_listener.waitForTransform("utm", "map", rospy.Time(), \
                                     rospy.Duration(5.0))
    except (ros_tf.LookupException):
        rospy.logerr('Cannot find utm-map tf')
        return None  
    (trans, rot) = tf_listener.lookupTransform('utm', 'map', rospy.Time(0))

    map_utm_tf_matrix = ros_tf_transformer.fromTranslationRotation(tuple(trans), tuple(rot))
    map_point = [x_map, y_map, 0, 1]
    utm_point = np.dot(map_utm_tf_matrix, map_point)
    
    UTM_ZONE_NUMBER = 48
    UTM_ZONE_LETTER = 'P'
    
    ll_point = utm.to_latlon(utm_point[0], utm_point[1], UTM_ZONE_NUMBER, UTM_ZONE_LETTER)
    ll_point = {'lat':ll_point[0], 'long':ll_point[1]}
    return(ll_point)
#============================================================================================  



#============================================================================================
'''   This function gets the GPS coordinate and convert it into a dict   '''
def gps_coordinate_callback(msg):
    #msg type: sensor_msgs/NavSatFix
    global gps_position
    gps_position = {'lat': msg.latitude, 'long': msg.longitude}
  
#============================================================================================



#============================================================================================
''' This funciton is to check whether the vehicle has bene already following a plan   '''
def move_base_stt_callback(msg):
    
    #msg type: actionlib_msgs/GoalID
    global busy_executing_plan
    try:
        moving_status = msg.status_list[0].status
        #moving_status == 1: executing plan
        #moving_status == 2: plan canceled
        #moving_status == 3: reach goal
        if (moving_status == 2) or (moving_status==3):
            busy_executing_plan = False
        if (moving_status == 1):
            busy_executing_plan = False
    except:
        pass   
      
#============================================================================================

    
    
#============================================================================================
'''   THIS WHOLE SECTION IS FOR PATH CALCULATION AND PATH_MSGS GENERATION!!!   '''
#--------------------------------------------------------------------------------------------
def utm_map_tf(utm_points):
    
    ros_tf_transformer = ros_tf.TransformerROS()
    try:
        tf_listener.waitForTransform("map", "utm", rospy.Time(), \
                                     rospy.Duration(5.0))
    except (ros_tf.LookupException):
        rospy.logerr('Cannot find utm-map tf')
        return None  
    (trans, rot) = tf_listener.lookupTransform('map', 'utm', rospy.Time(0))
    utm_map_tf_matrix = ros_tf_transformer.fromTranslationRotation(tuple(trans), tuple(rot))
    
    map_points = list()
    for i in utm_points:
        i = i + (0,1) #To fill z and w into the utm_points to become a (4,1) matrix
        map_points.append(list(np.dot(utm_map_tf_matrix, i)))
    
    return(map_points)
    
#--------------------------------------------------------------------------------------------
    
#--------------------------------------------------------------------------------------------
def calculate_heading(map_points):
    #Calculate heading in map
    #The first heading is based on current heading
    #Then heading is calculated based on the arctan of last and next map points
    
    heading = list()
    for i in range(len(map_points)):
        #------------------------------------------------------------------------------------
        if (i==0):
            try:
                tf_listener.waitForTransform("base_link", "map", rospy.Time(), \
                                             rospy.Duration(5.0))
            except (ros_tf.LookupException):
                rospy.logerr('Cannot find map->base_link tf')
                return None 
            (trans, rot) = tf_listener.lookupTransform('base_link', 'map', rospy.Time(0))
            heading.append(rot)
        #------------------------------------------------------------------------------------
        if (i!=0):
            #np.arctan2: Element-wise arc tangent of x1/x2 choosing the quadrant correctly.
            z_euler_heading = np.arctan2((map_points[i][1]-map_points[i-1][1]), \
                                         (map_points[i][0]-map_points[i-1][0]))
            loop_heading = ros_tf.transformations.quaternion_from_euler(0,0,z_euler_heading)
            heading.append(list(loop_heading))
        #------------------------------------------------------------------------------------
            
    return(heading)
    
#--------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------
def generate_path_msg(map_points, map_headings):
    
    path_msg = Path()
    path_msg.header = Header()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = 'map'
    
    #----------------------------------------------------------------------------------------
    for i in range(len(map_points)):
        single_point_pose = PoseStamped()
        single_point_pose.header = Header()
        single_point_pose.header.stamp = rospy.Time.now()
        single_point_pose.header.frame_id = "map"
        
        single_point_pose.pose.position.x = map_points[i][0]
        single_point_pose.pose.position.y = map_points[i][1]
        single_point_pose.pose.position.z = 0

        single_point_pose.pose.orientation.x = 0
        single_point_pose.pose.orientation.y = 0
        single_point_pose.pose.orientation.z = map_headings[i][2]
        single_point_pose.pose.orientation.w = map_headings[i][3]
        path_msg.poses.append(single_point_pose)
    #----------------------------------------------------------------------------------------
    
    return(path_msg)    
    
#--------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------
def calculate_path(gps_position, geo_goal):
    #Calculate path in MAP FRAME!
    
    #DANGEROUS ZONE!!!
    #PLEASE CHANGE YOUR OWN API KEY
    #OR NOT, PLEASE DO NOT SPAM THIS API KEY!
    API_KEY = "AIzaSyDc3VNCA56-e-qqel-CfB5rhEr1YwsaXqw"
    #current_pos, goal are dicts{'lat':, 'long':}
    url = "https://maps.googleapis.com/maps/api/directions/json?"
    url = url + "&origin=" + str(gps_position['lat']) + ',' + str(gps_position['long'])
    url = url + "&destination=" + str(geo_goal['lat']) + ',' + str(geo_goal['long'])
    url = url + "&key=" + API_KEY
    response = requests.get(url)

    if (response.ok == False):
        rospy.logerr("404 error to Google Map Direction API server")
        return(None)
    json_response = response.json()
    if (json_response['status'] != "OK"):
        rospy.logerr("Error in response from Google Map Direction API: %s" %json_response['status'])
        return(None)
 
    rospy.loginfo('Done getting navigation information from Google Map!')
    steps = json_response['routes'][0]['legs'][0]['steps']
    polylines = list()
    for i in steps:
        polylines.append(i['polyline']['points'])
    ll_points = list()
    for i in polylines:
        ll_points.extend(polyline.decode(i))
    utm_points = list()
    for i in ll_points:
        utm_points.append((utm.from_latlon(i[0],i[1]))[0:2])
    map_points = utm_map_tf(utm_points)

    #headings index is ordered respectively to map_points index
    map_headings = calculate_heading(map_points)
    map_path_msg = generate_path_msg(map_points, map_headings)
    
    return(map_path_msg)  
    
#--------------------------------------------------------------------------------------------
'''   END OF PATH CALCULATION AND PATH_MSGS GENERATION!!!   '''
#============================================================================================ 
    
   
 
#============================================================================================
'''   This function gets the goal as GPS coordinate 
      I the vehicle is busy following a plan: log an error
      If not: calculate a path, globalize it so the Path publisher can catch it  
'''
def geo_goal_callback(msg):
    
    #msg type: sensor_msgs/NavSatFix   
    global output_map_path_msg
    global busy_executing_plan
    
    if (busy_executing_plan == True) or (busy_executing_plan == None):
        rospy.logerr('Cancel last move_base command before execute new one!')
    if (busy_executing_plan == False):
        rospy.loginfo('New nav_goal received')
        geo_goal = {'lat': msg.latitude, 'long': msg.longitude}
        output_map_path_msg = calculate_path(gps_position, geo_goal)

#============================================================================================



#============================================================================================
def map_goal_callback(msg):
    
    #FRAME_ID OF MAP_GOAL: MAP   
    global geo_goal_pub
    global busy_executing_plan

    goal_x_map = msg.goal.target_pose.pose.position.x
    goal_y_map = msg.goal.target_pose.pose.position.y
    geo_goal   = map2ll(goal_x_map, goal_y_map)
    
    geo_goal_msg        = NavSatFix()
    geo_goal_msg.header = Header()
    geo_goal_msg.header.stamp  = rospy.Time.now()

    geo_goal_msg.latitude  = geo_goal['lat']
    geo_goal_msg.longitude = geo_goal['long']
    geo_goal_pub.publish(geo_goal_msg)
    
#============================================================================================

    

#============================================================================================
''' __MAIN__ '''

#Just for testing purpose!!!
global busy_executing_plan
busy_executing_plan = False
#--------------------------

rate = rospy.Rate(5)      

#--------------------------------------------------------------------------------------------        
move_base_stt_sub = rospy.Subscriber(move_base_stt_input, GoalStatusArray,    \
                                     move_base_stt_callback,     queue_size=1)
gps_sub           = rospy.Subscriber(gps_position_input, NavSatFix,           \
                                     gps_coordinate_callback,    queue_size=1)
geo_goal_sub      = rospy.Subscriber(geo_goal_input, NavSatFix,               \
                                     geo_goal_callback,          queue_size=1)
#CAUTION! map_goal_sub receive MoveBaseActionGoal from move_base node! 
#NOT FROM POINTSTAMPED RVIZ
map_goal_sub      = rospy.Subscriber(map_goal_input, MoveBaseActionGoal,      \
                                     map_goal_callback,           queue_size=1)
#--------------------------------------------------------------------------------------------  
path_pub          = rospy.Publisher(global_plan_output, Path,     queue_size=1)
#This geo_goal_pub actually just converts map_goal_sub in Map frame above and 
#Publish into a geo_goal_pub
geo_goal_pub      = rospy.Publisher(geo_goal_input, NavSatFix,    queue_size=1)
#--------------------------------------------------------------------------------------------  


while not rospy.is_shutdown():
    try:
        path_pub.publish(output_map_path_msg)
    except:
        pass
    rate.sleep()
