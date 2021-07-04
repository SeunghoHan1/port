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
import sensor.trajectory_communication as trajectory_communication
import sensor.state_communication as state_communication
from macaron_2.msg import erp42_write 
import vel_planner
import mathdir.pure_pursuit as pure_pursuit
from std_msgs.msg import Int32
from macaron_2.msg import Vel_ld
from macaron_2.msg import erp42_write 
import numpy as np
import rospy


def main():
    print(__file__ + " start!!")
    rospy.init_node('test_vel_planner', anonymous=True)    
    dataHub = sensor_data_communication.sensor_data_communicationer()
    stateHub = state_communication.finite_state_communicationer()
    planDataHub = trajectory_communication.trajectory_communicationer()
    rate=rospy.Rate(10) 
    velPlanner = vel_planner.V_P()
    # state= FSMachine.FSMState() 
    #vel_ld_pub = rospy.Publisher('vel_ld',Vel_ld,queue_size=1)
    #steer_pub = rospy.Publisher('steer',Int32,queue_size=1)
    write_pub = rospy.Publisher('erp42_write',erp42_write,queue_size=1)
    
    erp_write_msg = erp42_write()

    kappa = [0.1,0.1,0.1,0.1] #일단 실행하기 위한 임의 kappa
    # while not dataHub.kappa_ready():
    #     continue
    while not planDataHub.readySensor() and not stateHub.readySensor() :
        continue
    while not rospy.is_shutdown():
        # while not dataHub.readyerpspeed() :
        if planDataHub.readySensor() :
            kappa = planDataHub.get_current_kappa()
            print('kappa_on_vel',kappa)
            planDataHub.setSensorFlagOff()

        velPlanner.setup_v_gain(kappa)
        # car_state = state.getCurrentState()
        # read_speed = dataHub.get_current_car_speed()
        
        # run.setup_v_mission(car_state)
        # run.set_read_speed(read_speed)
        vel_infor = velPlanner.plan_vel()
        steer = pure_pursuit.get_steer(planDataHub.get_current_goal())
        erp_write_msg.write_E_stop = 0
        erp_write_msg.write_speed = vel_infor.speed
        erp_write_msg.write_brake = vel_infor.brake
        erp_write_msg.write_gear = vel_infor.gear
        erp_write_msg.write_steer = steer
        write_pub.publish(erp_write_msg)

        #vel_ld_pub.publish(vel_infor) #vel_ld 메세지를 publish.
        #steer_pub.publish(pure_pursuit.get_steer(planDataHub.get_current_goal()))

        # dataHub.setSensorFlagOff2()
        rate.sleep() 

if __name__ == '__main__':
    main()
    