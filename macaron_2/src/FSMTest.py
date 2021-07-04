#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 14 20:47:16 2020

@author: JHPark ^_^;
"""


import FSMachine
from FSMachine import CarStateE
import matplotlib.pyplot as plt
import sensor.sensor_data_communication as sensor_data_communication
import mathdir.cartesian_frenet_conversion as cartesian_frenet_conversion # 
import path_planner2
import path_planner_point_meter_ver
import math
import numpy as np
import rospy


show_animation = True
def main():
    print(__file__ + " start!!")
    rospy.init_node('steer_accel')
    dataHub = sensor_data_communication.sensor_data_communicationer()
    fsm = FSMachine.FSMachine()
    pthplanner = path_planner_point_meter_ver.path_planner("/home/park/catkin_ws/src/macaron_2/path/manhae1.npy")
    cartesianConverter = cartesian_frenet_conversion.CartesianFrenetConverter()
    rate = rospy.Rate(10)
    target_speed = 0
    csp = pthplanner.csp
    # dl = 0.5  # course tick
    WB = 1.04
    path = []
    
    while not rospy.is_shutdown() :
        while not dataHub.readySensor() :
            #print("not ready")
            continue
        x,y,yaw,v = dataHub.get_current_car_state()
        car_steer = dataHub.get_current_car_steer()
        fsm.update_state(v, signal = 1, current_pos=[x,y])
        if fsm.getCurrentState() == CarStateE.Start:
            target_speed = 30
        elif fsm.getCurrentState() == CarStateE.Cruise:
            target_speed = 10
        pthplanner.setObstacles(dataHub.get_current_obstacles())
        rs = pthplanner.getClosestSPoint(x,y)
        print("rs",rs)
        rx, ry = csp.calc_position(rs)
        ryaw = csp.calc_yaw(rs)
        rk = csp.calc_curvature(rs)
        rdk = csp.calc_d_curvature(rs)
        kappa = math.tan(np.deg2rad(car_steer))/WB
        d,dd,ddd = cartesianConverter.cartesian_to_frenet_only_d(rs, rx, ry, ryaw, rk, rdk, x, y, yaw, kappa)
        #print("d,dd,ddd",d,dd,ddd)
        pthplanner.updatePlanningParams(rs, d, dd, ddd, v)
        goal, kappa = pthplanner.select_path(target_speed)
        dataHub.setSensorFlagOff()
        
        area = 20.0
        if show_animation:
            plt.cla()
            plt.plot(csp.sx.y, csp.sy.y)
            ob = pthplanner.getObstacles()
            if len(ob) != 0 :
                plt.plot(ob[:, 0], ob[:, 1], "xk")
            plt.plot(path.x[1:], path.y[1:], "-or")
            plt.plot(x,y , "vc")
            plt.xlim(x[1] - area, x[1] + area)
            plt.ylim(y[1] - area, y[1] + area)
            plt.title("v[km/h]:" + str(v * 3.6)[0:4])
            plt.grid(True)
            plt.pause(0.0001)

        rate.sleep()

def main2():
    print(__file__ + " start!!")
    rospy.init_node('steer_accel')
    dataHub = sensor_data_communication.sensor_data_communicationer()
    fsm = FSMachine.FSMachine()
    pthplanner = path_planner2.path_planner("/home/park/catkin_ws/src/macaron_2/path/manhae1.npy")
    cartesianConverter = cartesian_frenet_conversion.CartesianFrenetConverter()
    rate = rospy.Rate(10)
    target_speed = 0
    csp = pthplanner.csp
    # dl = 0.5  # course tick
    WB = 1.04
    path = []
    
    while not rospy.is_shutdown() :
        while not dataHub.readySensor() :
            #print("not ready")
            continue
        x,y,yaw,v = dataHub.get_current_car_state()
        car_steer = dataHub.get_current_car_steer()
        fsm.update_state(v, signal = 1, current_pos=[x,y])
        if fsm.getCurrentState() == CarStateE.Start:
            target_speed = 30
        elif fsm.getCurrentState() == CarStateE.Cruise:
            target_speed = 10
        pthplanner.setObstacles(dataHub.get_current_obstacles())
        rs = pthplanner.getClosestSPoint(x,y)
        print("rs",rs)
        rx, ry = csp.calc_position(rs)
        ryaw = csp.calc_yaw(rs)
        rk = csp.calc_curvature(rs)
        rdk = csp.calc_d_curvature(rs)
        kappa = math.tan(np.deg2rad(car_steer))/WB
        d,dd,ddd = cartesianConverter.cartesian_to_frenet_only_d(rs, rx, ry, ryaw, rk, rdk, x, y, yaw, kappa)
        #print("d,dd,ddd",d,dd,ddd)
        pthplanner.updatePlanningParams(rs, d, dd, ddd, v)
        path = pthplanner.select_path(target_speed)
        dataHub.setSensorFlagOff()
        
        area = 20.0
        if show_animation:
            plt.cla()
            plt.plot(csp.sx.y, csp.sy.y)
            ob = pthplanner.getObstacles()
            if len(ob) != 0 :
                plt.plot(ob[:, 0], ob[:, 1], "xk")
            plt.plot(path.x[1:], path.y[1:], "-or")
            plt.plot(x,y , "vc")
            plt.xlim(x[1] - area, x[1] + area)
            plt.ylim(y[1] - area, y[1] + area)
            plt.title("v[km/h]:" + str(v * 3.6)[0:4])
            plt.grid(True)
            plt.pause(0.0001)

        rate.sleep()
    

    
if __name__ == '__main__':
    main()