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
from std_msgs.msg import Int32
import numpy as np
import rospy


def main():
    print(__file__ + " start!!")
    rospy.init_node('current_state')
    state_pub = rospy.Publisher('behavior_state', Int32, queue_size=1)
    dataHub = sensor_data_communication.sensor_data_communicationer()
    fsm = FSMachine.FSMachine()
    rate = rospy.Rate(2)
    state_msg = Int32()
    while not rospy.is_shutdown() :
        while not dataHub.readySensor() :
            #print("not ready")
            continue
        x,y,_,v = dataHub.get_current_car_state()
        fsm.update_state(v, signal = 1, current_pos=[x,y])
        if fsm.getCurrentState() == CarStateE.Start:
            target_speed = 30
        elif fsm.getCurrentState() == CarStateE.Cruise:
            target_speed = 10
        state_msg.data = fsm.getCurrentState().value
        state_pub.publish(state_msg)
        dataHub.setSensorFlagOff()
        rate.sleep()    

if __name__ == '__main__':
    main()
    