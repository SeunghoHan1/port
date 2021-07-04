#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 19 15:47:16 2020

@author: JHPark ^_^;
"""


import os, sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import FSMachine
from FSMachine import CarStateE
import matplotlib.pyplot as plt
import sensor.sensor_data_communication as sensor_data_communication
import sensor.state_communication as state_communication
import path_planner_point_meter_ver
import ld_planner
import numpy as np
import rospy

#Messages
from std_msgs.msg import Int32
from std_msgs.msg import Float64,Float64MultiArray
from geometry_msgs.msg import Pose

sys.path.append("../")


def main():
    print(__file__ + " start!!")
    rospy.init_node('path_planner')
    dataHub = sensor_data_communication.sensor_data_communicationer()
    stateHub = state_communication.finite_state_communicationer()
    x = 0
    y = 0
    heading = 0
    ld = 0
    obs_xy = []
    current_state = 0
    path_planner = path_planner_point_meter_ver.PathPlanner()

    goal_pub = rospy.Publisher('next_goal', Pose, queue_size=1)
    kappa_pub = rospy.Publisher('kappa',Float64MultiArray,queue_size=1)
    
    rate = rospy.Rate(5)
    # self.pub2path_tracker = rospy.Publisher('vel_ld',Vel_ld,queue_size=1)
    while not dataHub.readySensor() and not stateHub.readySensor() :
        continue
    while not rospy.is_shutdown() :
        
        if dataHub.readySensor():
            x, y, heading = dataHub.get_current_tm_and_heading()
            velocity = dataHub.get_current_car_speed()
            ld = ld_planner.setup_ld(velocity)
            obs_xy = dataHub.get_current_obstacles()
            path_planner.set_current_pose_head(x,y, heading)
            path_planner.set_current_ld_path_infor(ld)
            path_planner.set_obs_xy(obs_xy)
            dataHub.setSensorFlagOff()

        if stateHub.readySensor():
            current_state = stateHub.get_current_state()
            stateHub.setSensorFlagOff()
        
        # print ("x,y,heading", x, y, heading)
        
        # print ("ld", ld)
        
        # print ("obs_xy",obs_xy)
    
        


        goal, kappa = path_planner.select_path()
        # print("goal, kappa", goal, kappa)
        goal_pub.publish(goal)
        # print("kappa type:", type(kappa))
        kappa_pub.publish(kappa)
        
        rate.sleep()   

if __name__ == '__main__':
    main()
    