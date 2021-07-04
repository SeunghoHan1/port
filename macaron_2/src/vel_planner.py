#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import roslib
from math import cos, sin, pi, atan2
from std_msgs.msg import Float64,Int16,Float64MultiArray,Int32
from macaron_2.msg import Vel_ld, erp42_read # Vel_ld have ( speed/ brake / ld / steer / gear )
from time import sleep
import time
import path_planner_point_meter_ver
import sensor.sensor_data_communication as sensor_data_communication



# waypoint = np.load(file='/home/han/catkin_ws/src/macaron_2/path/manhae1.npy')
# sampled_path = np.load(file='/home/han/catkin_ws/src/macaron_2/path/manhae2.npy')
# arcleng_path = np.load(file= '/home/han/catkin_ws/src/macaron_2/path/manhae_arclength.npy')
x_offset = 955789 
y_offset = 1951239
KAPPA_MAX = 0.5112590689
V_MAX = 60  #현재 생각하는 최대속도, 버전마다 다르게 준비 할 수도 [speed는 실제 최대 200까지 가능]
V_START = 40
V_GAIN_MAX = V_MAX - V_START
V_GAIN_MIN = 10
KP = 0.5 # 원하는 출력에 빨리 도달하게 해줄겨 오차에 비례해서
KI = 0.5 # 오차상태 지속되면 값 커짐
KD = 0.3 # 속도의 급변에 저항[수렴 기울기 조절]
class V_P: #Vel_Planner

    vel_ld = Vel_ld() 
    
    def __init__(self):
        self.read_speed = 0
        self.v_gain = 0    
        self.v_mission = 0 
        self.P = 0 
        self.I = 0.0
        self.D = 0
        self.pid = 0
        self.last_read = 0 
        self.lasttime = 0 
        self.brake = 1

    def plan_vel(self):
        ld = 5
        now_speed = self.read_speed

        vel = V_START + self.v_gain + self.v_mission
        if self.v_mission != 0: a=0 # 감속 미션상황에서 pid 안함.
        else: a=1
        pid = self.control_vel_pid(vel) *a
        vel_pid = vel + int (pid)
        if vel_pid <= 0: vel_pid = 0 
        ld = self.setup_ld(now_speed)
        
        if vel >=200: vel = 200
        elif vel <= 0: vel = 0

        
        V_P.vel_ld.speed = vel_pid  # 출력을 줄 속도 값 ert_write_speed
        V_P.vel_ld.brake = self.brake # keyboard 제어 or mission
        V_P.vel_ld.ld = ld # 5
        V_P.vel_ld.gear = 0 #for keyboard 제어 or mission state  #GEAR = 0:전진, 1:중립, 2:후진
        # print(vel_ld)
        print("vel : %d " %vel) # PID 잘 되는지 확인
        print("vel_pid : %d " %V_P.vel_ld.speed)
        print("brake : %d " %self.brake)
        print("ld : %d " %V_P.vel_ld.ld)
        print("read_speed : %d " %now_speed)
        # print("self.read_speed : %f " %self.read_speed)
        return V_P.vel_ld
        
    def set_read_speed(self,read_speed):#erp42속도 read 값
        self.read_speed = read_speed

    def setup_ld(self,vel): #현재 속력(vel)에 따라서 ld 설정
        ld = float(vel)/22+2 # 최소 ld는 4m정도네 현재는 최대 8m정도 하고 싶음 [경로생성을 라이다 고려해서 8m 내외로 할 것 같아서 그거 고려해보자.]
        # ld = float(vel)/35+3 # 4~8m이긴 한데
        return ld
    
    def setup_v_gain(self,k): # 최대 V_MAX-xV_START 만큼 커질 수 있는 값이 v_gain 이다.
        cuvature_max = max(k)
        # cuvature_avg = sum(V_P.curvature.data) / len(V_P.curvature.data)
        curvature_choice = cuvature_max # cuvature_avg
        if curvature_choice >= 0.5: curvature_choice =0.5
        if curvature_choice <= 0.1: curvature_choice =0.1
            
        v_choice =range(V_GAIN_MAX,V_GAIN_MIN-1,-1) # 속도값 60 ~ 20값 선형적 비율로 선택 받을거임 41개
        #print(v_choice)
        index = int ( ( curvature_choice - 0.1 ) /  (0.5-0.1) *( len( v_choice)-1 )  )
        print("index", index)

        self.v_gain = v_choice[ index ] #  gain 값은 [V_GAIN_MAX~V_GAIN_MIN]
 
        ## 곡률       0.1        ~ 0.5        까지 subscribe 한다고 상정하고
        ## 속도 gain  V_GAIN_MAX ~ V_GAIN_MIN 까지 선형적으로 대응됨.

        ## v_gain= ( 10 ) / curvature_max  # 선형적이지 않아서 일단 보류하는 식
        
        
        # return v_gain
    
    def setup_v_mission (self,car_state):
    # Estop = -1
    # Ready_Start = 0
    # Start = 1
    # Cruise = 2
    # Ready_Light = 3
    # Stop_Red_Light = 4
    # Ready_Parking = 5
    # Parking = 6
    # Parking_Complete = 7
    
        # mission = car_state.value
        mission = 1

        if mission == 1:            # 일반상황: 1
            self.v_mission = 0
            self.brake = 1
        elif  mission == 2:
            self.v_mission = -20     # 감속상황 : 2
            self.brake =1
            self.I = 0
            self.D = 0
            self.P = 0
            self.pid = 0
        elif mission == 3 :         # 정지 상황 : 3  :동적 장애물 판단 상황 [ e_stop이랑은 별개로 일단 생각 ] 
            V_P.vel_ld.speed = 0
            self.v_mission = -50
            for i in range(1,200):  
                
                sleep(0.01)    
                self.brake = self.brake + 1
                self.plan_vel()
        elif mission == 4 :        # e_stop 상황 여기서 제어하려면, serial node 배열 변수 수정해 줘야함
            V_P.vel_ld.speed = 0
            self.v_mission = -200
            self.brake = 200
        
    def control_vel_pid(self,vel): #PID 제어
               
        dt = ( time.time()-self.lasttime ) 
        if dt > 1 :
            dt=0.02
        self.lasttime = time.time()

        error = vel - self.last_read

        self.P = KP * error
        self.I = float( self.I ) + ( error*KI ) *dt
        self.D =  ( self.last_read - self.read_speed ) *KD*dt
        self.last_read = self.read_speed
        self.pid =self.P +self.I + self.D
                    
        if self.pid > V_MAX - V_START: 
            self.pid = V_MAX - V_START
            self.I = float( self.I ) - ( error*KI ) *dt
        if  error  <= 5 :
            error = 0
            self.I = 0
            self.D = 0
            self.P = 0
            self.pid = 0
        print('error', error)
        return self.pid


def main():
    rospy.init_node('test_vel_planner', anonymous=True)
    rate=rospy.Rate(10) 
    run = V_P()
    dataHub = sensor_data_communication.sensor_data_communicationer()
    # state= FSMachine.FSMState() 
    pup_vel_ld = rospy.Publisher('vel_ld',Vel_ld,queue_size=1)
    kappa = [0.1,0.1,0.1,0.1] #일단 실행하기 위한 임의 kappa
    # while not dataHub.kappa_ready():
    #     continue

    while not rospy.is_shutdown():
        # while not dataHub.readyerpspeed() :
            if dataHub.kappa_ready() :
                kappa = dataHub.get_crrent_kappa()
                print('kappa_on_vel',kappa)
                dataHub.setSensorFlagOff3()
    
            
            run.setup_v_gain(kappa)
            # car_state = state.getCurrentState()
            # read_speed = dataHub.get_current_car_speed()
            
            # run.setup_v_mission(car_state)
            # run.set_read_speed(read_speed)
            vel_infor = run.plan_vel()
            pup_vel_ld.publish(vel_infor) #vel_ld 메세지를 publish.
    
            # dataHub.setSensorFlagOff2()
            rate.sleep()


## start code   
if __name__ == '__main__':
    main()
    