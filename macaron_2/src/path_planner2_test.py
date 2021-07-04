#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Aug  6 18:19:56 2020

@author: Elitebook 8570w
"""

import rospy
import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import path_planner2
from math import tan, atan2, cos, sin, pi
import sensor_only_for_test
import mathdir.cartesian_frenet_conversion as cartesian_frenet_conversion
#from IPython import display

show_animation = True
WB = 1.04

def main():

    print(__file__ + "start!!")
    dataHub = sensor_only_for_test.tempDataHub()
    cartesianConverter = cartesian_frenet_conversion.CartesianFrenetConverter()
    rospy.init_node("path_planner")
    #next_pub = rospy.Publisher("selected_path", Frenet_path_msg, queue_size=1)
    #as
    # way points
    #wx = [0.0, 10.0, 20.5, 35.0, 70.5]
    #wy = [0.0, -6.0, 5.0, 6.5, 0.0]
    # obstacle lists
    #ob = np.array([[955793.371085,1951231.241498],
    #               [955802.831106,1951256.440183],
    #               [955781.147602,1951255.923337],
    #               [955782.256198,1951253.487930],
    #               ])
    


    # initial state
    pthplanner = path_planner2.path_planner('/home/park/catkin_ws/src/macaron_2/path/manhae1.npy')
    area = 20.0  # animation area length [m]
    csp = pthplanner.csp
    #next_path = Frenet_path_msg()
    #for i in range(500): # search depth
    rate = rospy.Rate(10)
    #i = 0
    while not rospy.is_shutdown():
        #i = i + 1
        #print("rospy is running")
        while not dataHub.readySensor():
            continue
        x,y,yaw,v = dataHub.get_current_car_state()
        car_steer = dataHub.get_current_car_steer()
        path = pthplanner.select_path(30)
        if path is None:
            print("no more path generated")
            break
        #print(path)
        """
        s0 = path.s[1]
        c_d = path.d[1]
        c_d_d = path.d_d[1]
        c_d_dd = path.d_dd[1]
        c_speed = path.s_d[1]
        print(path.s_d)
        """
        c_speed = v
        rs = pthplanner.getClosestSPoint(x,y)
        print("rs",rs)
        rx, ry = csp.calc_position(rs)
        ryaw = csp.calc_yaw(rs)
        rk = csp.calc_curvature(rs)
        rdk = csp.calc_d_curvature(rs)
        kappa = math.tan(car_steer)/WB
        d,dd,ddd = cartesianConverter.cartesian_to_frenet_only_d(rs, rx, ry, ryaw, rk, rdk, x, y, yaw, kappa)
        
        pthplanner.updatePlanningParams(rs, d, dd, ddd, c_speed)
        dataHub.setSensorFlagOff()
        
        #next_path.x=path.x
        #next_path.y=path.y  
        #next_path.yaw=path.yaw
        #next_path.c=path.c
        #next_path.speed = c_speed
        
        # publish goal
        
        #next_pub.publish(next_path)
        
        # goal check
        goalx, goaly = pthplanner.getGoalXY()
        tx, ty = pthplanner.getTxTy()
        #print("goal info", path.x[1], goalx, path.y[1], goaly)
        print("c_speed",c_speed*3.6)
        if np.hypot(x - goalx, y - goaly) <= 1:
            print("Goal")
            break

        # animation part
        """
        if show_animation:
            plt.cla()
            plt.plot(tx, ty)
            ob = pthplanner.getObstacles()
            if len(ob) != 0 :
                plt.plot(ob[:, 0], ob[:, 1], "xk")
            plt.plot(path.x[1:], path.y[1:], "-or")
            plt.plot(path.x[1], path.y[1], "vc")
            plt.xlim(path.x[1] - area, path.x[1] + area)
            plt.ylim(path.y[1] - area, path.y[1] + area)
            plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
            plt.grid(True)
            plt.pause(0.0001)
            """
        if show_animation:
            plt.cla()
            plt.plot(tx, ty)

            ob = pthplanner.getObstacles()
            if len(ob) != 0 :
                plt.plot(ob[:, 0], ob[:, 1], "xk")

            plt.plot(path.x[1], path.y[1], "vc")
            plt.plot(path.x[1:], path.y[1:], "-or")

            for (ix, iy) in zip(path_planner2.faTrajX, path_planner2.faTrajY):
                #pdb.set_trace()
                plt.plot(ix[1:], iy[1:], '-', color=[0.5, 0.5, 0.5])
            path_planner2.faTrajX = []
            path_planner2.faTrajY = []

            for (ix, iy) in zip(path_planner2.faTrajCollisionX, path_planner2.faTrajCollisionY):
                #pdb.set_trace()
                plt.plot(ix[1:], iy[1:], 'rx')
            path_planner2.faTrajCollisionX = []
            path_planner2.faTrajCollisionY = []
            #pdb.set_trace()
            for fp in path_planner2.fpplist:
                #pdb.set_trace()
                plt.plot(fp.x[1:], fp.y[1:], '-g')
            path_planner2.fpplist = []

            #pdb.set_trace()
            for (ix, iy) in zip(path_planner2.faObCollisionX, path_planner2.faObCollisionY):
                #pdb.set_trace()
                plt.plot(ix, iy, 'oy')
            path_planner2.faObCollisionX = []
            path_planner2.faObCollisionY = []

            plt.plot(path.x[1:], path.y[1:], "-ob")

            plt.xlim(path.x[1] - area, path.x[-1] + area)
            plt.ylim(path.y[1] - area, path.y[-1] + area)
            plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
            plt.grid(True)
            plt.pause(0.0001)
            #display.clear_output(wait=True)
            #fig = pl.gcf()
            #display.display(fig)
        rate.sleep()

    print("Finish")
    if show_animation:
        plt.grid(True)
        plt.pause(0.0001)
        plt.show()


if __name__ == '__main__':
    main()