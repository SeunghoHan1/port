#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import roslib
from math import cos, sin, pi, atan2
from std_msgs.msg import Float64,Int16
from macaron_2.msg import Vel_ld, erp42_read # Vel_ld have ( speed/ brake / ld / steer / gear )
import time



# waypoint = np.load(file='/home/han/catkin_ws/src/macaron_2/path/manhae1.npy')
# sampled_path = np.load(file='/home/han/catkin_ws/src/macaron_2/path/manhae2.npy')
# arcleng_path = np.load(file= '/home/han/catkin_ws/src/macaron_2/path/manhae_arclength.npy')
x_offset = 955789 
y_offset = 1951239
v_max = 100  #현재 생각하는 최대속도, 버전마다 다르게 준비 할 수도
v_start = 20
v_gain = 0    
v_mission = 0 
lasttime = 0 
last_read = 0 
read_speed= 0
erp42_read_sub = erp42_read()
P = 0 
I = 0
D = 0
pid=0

class Vel_Planner:
    def __init__(self): 
        self.path_planner_pub_ld = rospy.Publisher('vel_ld',Vel_ld,queue_size=1)
        self.sub_erp42_read = rospy.Subscriber("erp42_read", erp42_read, self.sub_erpread,queue_size=1)
        # self.sub_slected_path_k = rospy.Subscriber("erp42_write", erp42_read, self.setup_v_gain, queue_size=1) # path_planner로 부터 선택된 경로의 ld까지의 곡률 배열을 받아옴.
        self.run()

    def plan_vel(self):
        global pid
        # v_gain = self.setup_v_gain([1,2,3]) # 곡률 받고나면 계산할겨
        # v_mission = self.setup_v_mission([situation]) # 미션 받고 감속 하는거
        vel = v_start + v_gain + v_mission       
        pid = self.control_vel_pid(vel)
        vel_pid = vel + int (pid)

        ld = self.setup_ld(vel)
        vel_ld = Vel_ld()
        vel_ld.speed = vel_pid #vel_pid  # 출력을 줄 속도 값 ert_write_speed
        vel_ld.brake = 0 # keyboard 제어 or mission
        vel_ld.ld = ld
        vel_ld.gear = 0 # keyboard 제어 or mission
        # print(vel_ld)
        print("vel : %f " %vel)
        print("vel_pid : %f " %vel_ld.speed)
        # print("read_speed : %f " %read_speed)
        self.path_planner_pub_ld.publish(vel_ld)    
    def sub_erpread(self,erp42_read_sub):
        global read_speed
        read_speed = erp42_read_sub.read_speed      

    def setup_ld(self,vel): #현재 원하는 속력(vel)에 따라서 ld 설정하는게 맞음
        ld = float(vel)/22+2 # 최소 ld는 4m정도네 현재는 최대 11m
        return ld
    
    def setup_v_gain(self,k): # 최대 v_max-v_start 만큼 커질 수 있는 값이 v_gain 이다.
        # curvature = message()
        # cuvature = max(message.k)
        # 보내지는 k (곡률)의 평균 or 최대값이 실제로 어느정도 범위 인지 확인해서 0~0.5라고하면 상수/곡률최대0.5112590689  = 최소gain 값(임의 50)이 나오도록 상수 지정 해야함
                                                                                        #  상수/곡률최소0.01          = 최대 gain값 (v_max - v_start) 나오도록 상수 지정
        #     [0.01 : 0.5] = [v_max-v_start : 50 ] 이거 대응되는 식 만들어야함.
        # v_gain= ( 50 ) / k
        v_gain=int (0)
        return v_gain
    
    def setup_v_mission (self,mission):
        v_mission =0
        return v_mission
    def control_vel_pid(self,vel): #
        global lasttime ,last_read, read_speed ,P, I ,D, v_max
        kp = 0.5 # 원하는 출력에 빨리 도달하게 해줄겨 오차에 비례해서
        ki = 0.3 # 오차상태 지속되면 값 커짐
        kd = 0.3 # 속도의 급변에 저항
        dt = ( time.time()-lasttime ) 
        if dt > 1 :
            dt=0.02
        lasttime = time.time()
        
        error = vel - last_read

        P = kp * error
        I = I + ( error*ki ) *dt
        D =  ( last_read - read_speed ) *kd*dt
        last_read =read_speed
        
        pid = P + I + D
        
            
        if pid > v_max - v_start:
            pid = v_max - v_start
        if  error  <= 5 :
            error = 0
            I = 0
            D = 0
            P = 0
            pid = 0
        return pid

    
    def run(self):
        rate=rospy.Rate(50) 
        while not rospy.is_shutdown():
            self.plan_vel()
            rate.sleep()

def main():
    rospy.init_node('vel_planner', anonymous=True)
    Vel_Planner() 


## start code   
if __name__ == '__main__':
    main()
