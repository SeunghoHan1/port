#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Aug  9 19:45:50 2020

@author: JHPark ^^;
"""
import numpy as np
import enum
#import sensor_data_communication

class CarStateE(enum.Enum) : 
    Estop = -1
    Ready_Start = 0
    Start = 1
    Cruise = 2
    Ready_Light = 3
    Stop_Red_Light = 4
    Ready_Parking = 5
    Parking = 6
    Parking_Complete = 7
    # dynamic_obstacle
    # crosswalk_stop
    # 
    
class FSMState : 
    def __init__(self, curState = CarStateE.Ready_Start):
        self.curState = curState
        self.possibleState = self.getPossibleStates(curState)
        
    def getPossibleStates(self, curState):
        return {CarStateE.Ready_Start : [CarStateE.Start],
                CarStateE.Start : [CarStateE.Cruise],
                CarStateE.Cruise : [CarStateE.Ready_Light, CarStateE.Ready_Parking, CarStateE.Estop, CarStateE.Stop_Red_Light],
                CarStateE.Ready_Parking : [CarStateE.Parking, CarStateE.Cruise, CarStateE.Estop],
                CarStateE.Ready_Light : [CarStateE.Stop_Red_Light, CarStateE.Cruise, CarStateE.Estop],
                CarStateE.Estop : [CarStateE.Ready_Start],
                CarStateE.Parking : [CarStateE.Parking_Complete],
                CarStateE.Stop_Red_Light : [CarStateE.Ready_Start],
                CarStateE.Parking_Complete : [CarStateE.Ready_Start]
                }[curState]
    
    def isPossibleToTrans(self, candidateState):
        return any([possible == candidateState for possible in self.possibleState])
    
    def trans2NextState(self, nextState):
        if any([possible == nextState for possible in self.possibleState]):
            self.curState = nextState
            self.possibleState = self.getPossibleStates(self.curState)
            return True
        return False
    
    def trans2Estop(self) :
        self.curState = CarStateE.Estop
        self.possibleState = self.getPossibleStates(self.curState)
    
    def getCurrentState(self):
        return self.curState
    
    
    
class FSMachine:
    CRUISE_VELOCITY = 10 / 3.6
    READY_LIGHT_VELOCITY = 5 / 3.6
    READY_PARKING_VELOCITY = 2.5 / 3.6
    PARKING = 0
    LIGHT = 1
    LIGHT_RED = 2
    def __init__(self):
        self.stateHub = FSMState()
        #self.curState = stateHub.cur_state
        
    def update_state(self, velocity, recognized_objects = [], signal = 0, sensor_data = [], current_pos = []):
        curState = self.stateHub.curState
        
        # when sensor or other settings are failed
        if signal == -1 :
            self.stateHub.trans2Estop()
            return self.stateHub.curState
        candidate_states = []
        if len(recognized_objects) != 0 :
            candidate_states = self.__statesRecognizedObjects(recognized_objects)
        
        # when sensor or other settings are ready
        if curState == CarStateE.Ready_Start :
            if signal == 1 :
                self.stateHub.trans2NextState(CarStateE.Start)
                return self.stateHub.curState
        # when velocity reaches cruise velocitys
        elif curState == CarStateE.Start :
            if velocity >= FSMachine.CRUISE_VELOCITY :
                self.stateHub.trans2NextState(CarStateE.Cruise)
                return self.stateHub.curState
        # when object is recognized, it transites to Ready_Light, Ready_Parking, Stop_Red_Light 
        elif curState == CarStateE.Cruise :
            if len(candidate_states) > 0 :
                state = self.__chooseStateInCandidateStates(candidate_states)
                self.stateHub.trans2NextState(state)
                return self.stateHub.curState
        # when traffic light is Red, it transites to Stop_Red_Light
        # when car pass through traffic light, it returns to cruise mode
        elif curState == CarStateE.Ready_Light :
            if len(candidate_states) > 0 :
                if any([can_state == CarStateE.Stop_Red_Light for can_state in candidate_states]):
                    self.stateHub.trans2NextState(CarStateE.Stop_Red_Light)
                    return self.stateHub.curState
                elif not any([can_state == CarStateE.Ready_Light for can_state in candidate_states]):
                    self.stateHub.trans2NextState(CarStateE.Cruise)
                    return self.stateHub.curState
        # when light is red, it stops, when light is green(signal 2) it transites to ready_start
        elif curState == CarStateE.Stop_Red_Light :
            if not any([can_state == CarStateE.Stop_Red_Light for can_state in candidate_states]):
                self.stateHub.trans2NextState(CarStateE.Ready_Light)
                return self.stateHub.curState
            if signal == 2 :
                self.stateHub.trans2NextState(CarStateE.Ready_Start)
                return self.stateHub.curState
        # when detecting parking signal, car finds suitable place to park
        # when car found parking place and path(signal 3), gose to Parking state
        elif curState == CarStateE.Ready_Parking :
            if signal == 3:
                self.stateHub.trans2NextState(CarStateE.Parking)
                return self.stateHub.curState
        # when parking is done(signal 4), gose to parking complete state
        elif curState == CarStateE.Parking :
            if signal == 4:
                self.stateHub.trans2NextState(CarStateE.Parking_Complete)
                return self.stateHub.curState
        # when parking is done and wait for mission_seconds, 
        # it returns to its initial position(signal 5) and gose to ready start state  
        elif curState == CarStateE.Parking_Complete :
            if signal == 5:
                self.stateHub.trans2NextState(CarStateE.Ready_Start)
                return self.stateHub.curState
        # wait for issues get fixed
        elif curState == CarStateE.Estop :
            if signal == 1:
                self.stateHub.trans2NextState(CarStateE.Ready_Start)
                return self.stateHub.curState
            
        return curState
                
        #for cur_object in recognized_objects :
        #    objectType = cur_object.type
    
    def __statesRecognizedObjects(self, recognized_objects):
        candidate_states = []
        for cur_object in recognized_objects :
            if cur_object == FSMachine.LIGHT : 
                candidate_states.append(CarStateE.Ready_Light)
            if cur_object == FSMachine.PARKING :
                candidate_states.append(CarStateE.Ready_Parking)
            if cur_object == FSMachine.LIGHT_RED : 
                candidate_states.append(CarStateE.Stop_Red_Light)
        return candidate_states
    
    def __chooseStateInCandidateStates(self,candidate_states):
        cur_point = candidate_states[0]
        for can_state in candidate_states:
            if can_state == CarStateE.Stop_Red_Light :
                return CarStateE.Stop_Red_Light
            elif can_state == CarStateE.Ready_Parking :
                cur_point = CarStateE.Ready_Parking
            elif cur_point != CarStateE.Ready_Parking and can_state == CarStateE.Ready_Light:
                cur_point = CarStateE.Ready_Light
            #else :
            #    cur_point = can_state 
        return cur_point
    
    def getCurrentState(self):
        return self.stateHub.curState
    #def findNextStateInMultipleState(self, possible_states, )
            
def main() :
    testFsm = FSMState()
    print(testFsm.getCurrentState())
    print(testFsm.possibleState)
    testFsm.trans2NextState(CarStateE.Start)
    print(testFsm.getCurrentState())
    print(testFsm.possibleState)
    testFsm.trans2NextState(CarStateE.Cruise)
    print(testFsm.getCurrentState())
    print(testFsm.possibleState)
    fsm = FSMachine()
    for i in range(0,1000):
        fsm.update_state(i/36.0)
        print(fsm.getCurrentState())
    
    

if __name__ == '__main__':
    main()
    
    
    
    
    