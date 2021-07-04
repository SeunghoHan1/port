#!/usr/bin/env python
#-*-coding:utf-8-*-

# Python packages
import rospy
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import distance
from math import sin, cos, tan, pi, atna2
import sensor_data_communication

# Messages
from geometry_msgs.msg import Pose
from macaron_2.msg import erp42_write

# visuallizer
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3,PoseStamped
from std_msgs.msg import ColorRGBA, Header
from tf.transformations import quaternion_from_euler

# parking_information
PARKING_BASE_FRAME = np.load(file='/home/zzino/catkin_ws/src/macaron_2/path/parking.npy')
PARKING_BASE_FRAME = np.transpose(PARKING_BASE_FRAME)
BASE_FRAME_POINT_NUM = PARKING_BASE_FRAME.shape[0]
PARKING_INDEX = 0
PARKING_POSITION = [[[955780.3221354599,1951281.664919219]],
                    [[0,0]],
                    [[0,0]],
                    [[0,0]]]             # 인덱스 순서 : parking_index, x, y # 3차원 배열
PARKING_HEADING = 7.25 # 주차장 heading 32 도인데 실제 heading이랑 맞춰봐야됨


class Parking:
    def __init__(self):
        self.carstate_pub = rospy.Publisher('erp42_write', erp42_write, queue_size=1)
        self.si = 0
        self.sl = []
        self.turning_point = np.empty(1,3)
        self.turning_point_tm = np.empty(1,3) # x ,y
        self.x_offset = 0
        self.y_offset = 0
        self.goal_sequence = 0
        self.park_index = 0
        self.parking_state = 0
        self.time_state = 0
        self.obs_xy = []
        self.theta_start = heading # TM좌표 기준 차가 보고있는 초기 방향
        self.theta_goal = 0 # TM 좌표 기준 차가 park_goal에 도착했을때 바라보고있어야할방향
    def set_obs_xy(self,obs_xy):
        self.obs_xy = obs_xy
    def set_current_pose_head(self,x,y,heading):
        self.heading = heading
        self.current_pose = [x,y]
    def set_current_ld_path_infor(self,ld):
        self.ld = ld

    def get_current_sequence(self,s,x,y):
        position = [x, y]
        search = 0
        diff = 100
        
        while(abs(diff) > 0.001):
            diff = distance.euclidean(position, PARKING_BASE_FRAME[s + 1, :]) - \
                   distance.euclidean(position, PARKING_BASE_FRAME[s    , :])
            search += 1
            alpha = 1/(1+search/50)
            descendent = -alpha * diff

            if -1<descendent and descendent<0: descendent = -1                                                                                                                                                                                                                                                                                 
            elif 0<descendent and descendent<1: descendent = 1
            elif isnan(descendent): descendent = 0
            s += int(descendent)
            if s > base_length - 1: 
                s = base_length - 1
                break
            elif s < 0: 
                s = 0
                break
            
            if search == 50:
                break

        s = int(s)

        return s

    def pub_next_goal(self,sequence):
        goal = Pose()
        heading = self.heading
        next_sequence = sequence + 30
        x_tm_next, y_tm_next = PARKING_BASE_FRAME[next_sequence]
        T = [[cos(self.heading), sin(self.heading), -self.current_pose[0]*cos(self.heading)-self.current_pose[1]*sin(self.heading)],\
            [-sin(self.heading),  cos(self.heading), self.current_pose[0]*sin(self.heading)-self.current_pose[1]*cos(self.heading)],\
            [           0,             0, 1]]
        
        self._sl = np.dot(T,np.transpose([x_tm_next, y_tm_next, 1]))
        goal.position.x = self._sl[0]
        goal.position.y = self._sl[1]

        return goal    


    def calculate_offset_and_turning_point(self,kappa,theta): # 
        current_pose = self.current_pose
        R = (1/kappa)
        theta = theta * (pi/180)
        self.x_offset = R * sin((pi/2) - theta)
        self.y_offset = R - R*cos((pi/2) - theta)
        T = [[cos(PARKING_HEADING), -sin(PARKING_HEADING), PARKING_POSITION[PARKING_INDEX,0,0]], \
             [sin(PARKING_HEADING), cos(PARKING_HEADING), PARKING_POSITION[PARKING_INDEX,0,1]], \
             [         0          ,           0         ,             1        ]]
        
        self.turning_point = [PARKING_POSITION[PARKING_INDEX,0,0] - self.x_offset, PARKING_POSITION[PARKING_INDEX,0,1] - self.y_offset, 1]
        self.turning_point_tm = np.dot(T, np.transpose(self.turning_point))
        self.turning_point_tm = np.delete(self.parking_turning_point, (2), axis = 1)    


    def park_algorithm(self):
        erp = erp42_write()
        
        current_pose = self.current_pose
        heading = self.heading
        
        if distance.euclidean(current_pose,self.turning_point_tm) < 0.2 :
            base_frame_heading = heading
            self.parking_state = 1
        
        if self.parking_state == 1:
            erp.write_steer = 2000 
            erp.write_speed = 20 
            erp.write_E_stop = 0
            erp.write_gear = 0
            erp.write_brake = 1
            self.carstate_pub.publish(erp) 
        
            if heading - 0.1 < PARKING_HEADING and PARKING_HEADING < heading + 0.1 :
                self.parking_state = 2
        
        elif self.parking_state == 2:
            erp.write_steer = 0 
            erp.write_speed = 20 
            erp.write_E_stop = 0
            erp.write_gear = 0
            erp.write_brake = 1
            self.carstate_pub.publish(erp) 
            
            if self.time_state == 0 :
                prev_time = rospy.get_rostime()
                self.time_state == 1
            cur_time = rospy.get_rostime()
            if cur_time.secs- prev_time.secs > 3 :
                self.parking_state = 3
        
        elif self.parking_state == 3:
            e.stop = 1
            if self.time_state == 1:
                prev_time = rospy.get_rostime()
                self.time_state = 2
            cur_time = rospy.get_rostime()
            if cur_time.secs - prev_time.secs > 10: 
                self.parking_state = 4
        
        elif self.parking_state == 4:
            e.gear = 1
            speed = 20
            steer = 0
            if self.time_state == 2:
                prev_time = rospy.get_rostime()
                self.time_state = 3
            cur_time = rospy.get_rostime()
            if cur_time.secs - prev_time.secs > 3:
                self.parking_state = 5
        elif self.parking_state == 5:
            e.gear = 1
            speed = 20
            steer = 2000
            if heading - 0.1 < base_frame_heading and base_frame_heading < heading + 0.1
                self.parking_state =6
                print("parking_complete")
        elif self.parking_state == 6 :
            e.stop = 1


def main():
    # goal_pub = rospy.Publisher('next_goal', Pose, queue_size=1)
    rospy.init_node('parking')
    
    goal_pub = rospy.Publisher('next_goal', Pose, queue_size=1) # 위치?

    park = Parking()
    dataHub = sensor_data_communication.sensor_data_communicationer()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        while not dataHub.readySensor
            i = 0
        x,y,heading = dataHub.get_current_tm_and_heading()
        ld = dataHub.get_current_ld()
        obs_xy = dataHub.get_current_obstacles()

        park.set_current_pose_head(x,y,heading)
        park.set_current_ld_path_infor(ld)
        park.set_obs_xy(obs_xy)
        
        si = park.get_current_sequence()

        goal_pub.publish(goal) # 위치?
        si = park.get_current_sequence()
        while park.pub_next_goal(si) #  parking용 전역경로 추종하는 와중에 함수실행?
               
            park.calculate_offset(0.511,32)
            park.park_algorithm() # 이게맞나?




if __name__ == '__main__':
    main()