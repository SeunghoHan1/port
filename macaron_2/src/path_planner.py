#!/usr/bin/env python
#-*-coding:utf-8-*-

# Python packages
import rospy
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import distance
from math import sin, cos, tan, pi, isnan
import sensor.sensor_data_communication as sensor_data_communication
import ld_planner
from time import sleep 

# Messages
from std_msgs.msg import Float64,Float64MultiArray
from geometry_msgs.msg import Pose
from macaron_2.msg import Vel_ld

# visuallizer
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3,PoseStamped
from std_msgs.msg import ColorRGBA, Header
from tf.transformations import quaternion_from_euler



# Load base frame(x, y, theta, kappa)
BASE_FRAME_XY_DT = np.load(file='/home/han/catkin_ws/src/macaron_2/path/K-CITY-garage-0.1m.npy')       #RESOLUTION = 0.1m sequence 찾기 위한 용도.
BASE_FRAME_XY_DT = np.transpose(np.array([BASE_FRAME_XY_DT[:, 0], BASE_FRAME_XY_DT[:, 1]]))
# BASE_FRAME_XY_DT = np.load(file='/home/park/catkin_ws/src/macaron_2/path/manhae_base_true_sample_for_test.npy')       #RESOLUTION = 0.1m sequence 찾기 위한 용도.

BASE_FRAME = np.load(file='/home/han/catkin_ws/src/macaron_2/path/K-CITY-garage-0.5m.npy')                       #RESOLUTION = 1m
# BASE_FRAME = np.load(file='/home/park/catkin_ws/src/macaron_2/path/manhae_base_frame.npy')                       #RESOLUTION = 1m
# BASE_FRAME = np.load(file='/home/han/catkin_ws/src/macaron_2/path/manhae_base_frame_0.5m.npy')                       #RESOLUTION = 0.5m 
BASE_XY = np.transpose(np.array([BASE_FRAME[:, 0], BASE_FRAME[:, 1]])) # ( 115*N , 2)
BASE_THETA = np.transpose(np.array([BASE_FRAME[:, 2]])) # ( 115*N , 1)
BASE_KAPPA = np.transpose(np.array([BASE_FRAME[:, 3]])) # ( 115*N , 1) 

# Path Property
FIX_PATH_LENG = 10 # extend path (m)
PATH_INDEX = 3 #후보 경로 몇개니 ?
RESOLUTION = 0.5 # 좌표 간격 meter(s 하나당)
N= int(1/RESOLUTION) # path_point = path_length * N +1 로 정의 할거임.  현재 N은 2
INTERVAL = 0.7 # between candidates 

# Cost Weight
W_SAFETY = 5
W_OFFSET = 1
W_CONSISTENCY = 0.5

x_offset = BASE_XY[0,0]
y_offset = BASE_XY[0,1]

# Car Property
GPS_LIDAR_OFFSET = 1.3

class PathPlanner: # path planner

    def __init__(self):
        self.heading = 0.0
        self.obs_xy = []
        self.current_pose = [x_offset,y_offset]
        self.ld = 5
        self.path_length = int (self.ld)
        self.path_point = self.path_length *N +1
        self.fix_path_point = FIX_PATH_LENG *N +1
        self.cost = np.zeros(PATH_INDEX)
        self.si = 0
        self.prev_si = 0
        self.sl = []
        self.selected_index = 0
        self.kappa = Float64MultiArray()
        self.qf = INTERVAL*np.array([(x - PATH_INDEX) for x in range(0, PATH_INDEX * 2 + 1)])
        self.prev_sl_selected_path = np.zeros((self.fix_path_point)) # for consistency cost
        self.candidate_path = np.zeros([PATH_INDEX, self.path_point]) # SL frame [offest array]
        self.candidate_path_xy = np.zeros([PATH_INDEX, self.path_point, 2]) # Universial frame
        self.obs_candidate_path = np.zeros([PATH_INDEX+1, self.fix_path_point]) # for safety cost
        self.safe_candidate_path = np.zeros([PATH_INDEX, self.fix_path_point]) 
        self.safe_candidate_path_xy = np.zeros([PATH_INDEX, self.fix_path_point, 2]) 
        self.cand_path = rospy.Publisher('candi', Marker, queue_size=1)
        self.obs_plot = rospy.Publisher('obs', Marker,queue_size=1)

    def set_current_pose_head(self, x, y, heading):
        self.heading = heading
        self.current_pose = [x, y]
    def set_current_ld_path_infor(self, ld):
        self.ld = ld
        self.path_length = int (self.ld) # 2 ~ 10
        self.path_point = self.path_length *N +1
        self.candidate_path = np.zeros([PATH_INDEX, self.path_point]) # SL frame [offest array]
        self.candidate_path_xy = np.zeros([PATH_INDEX, self.path_point, 2]) # Universial frame

    def set_obs_xy(self,obs_xy):
        self.obs_xy = obs_xy

    def generate_path(self):
        a1 = np.zeros(PATH_INDEX* 2 + 1) # Coefficient 1 for candidate path function
        a2 = np.zeros(PATH_INDEX* 2 + 1) # Coefficient 2 for candidate path function
        self.si, qi = self.localize2sl(self.si, self.current_pose[0], self.current_pose[1]) # Current si, qi
         # Candidates s #//
        s = range(0,self.path_point)  # 원래는 path 길이 만큼 생성
        print('si',self.si)
        theta = self.heading - BASE_THETA[self.si]
        # Generate Path
        for i in range(0, PATH_INDEX* 2 + 1):
            for j in range(0, self.path_point):
                a1[i] = (self.path_length*tan(theta) - 2*(self.qf[i] - qi)) / pow(self.path_length, 3)
                a2[i] = (self.qf[i] - qi) / pow(self.path_length, 2)
                k = int(np.modf([i/2.0])[1][0])
                if i % 2 == 0 : # 
                    self.obs_candidate_path[k, j] = pow(s[j]*RESOLUTION - self.path_length, 2) * (a1[i]*s[j]*RESOLUTION - a2[i]) + self.qf[i] # for Safety cost
                    if j+1 == self.path_point:
                        for n in range(0, self.fix_path_point - self.path_point):
                            self.obs_candidate_path[k,j+n+1] = self.obs_candidate_path[k,j]
                else :
                    self.candidate_path[k, j] = self.safe_candidate_path[k,j] = pow(s[j]*RESOLUTION - self.path_length, 2) * (a1[i]*s[j]*RESOLUTION - a2[i]) + self.qf[i]
                    self.candidate_path_xy[k, j] = self.safe_candidate_path_xy[k,j] = self.localize2xy(self.si + s[j], self.candidate_path[k, j]) 
                    if j+1 == self.path_point:
                        for n in range(0, self.fix_path_point - self.path_point):
                            self.safe_candidate_path[k,j+n+1] = self.safe_candidate_path[k,j]
                            self.safe_candidate_path_xy[k,j+n+1] = self.localize2xy(self.si + s[j] + (n+1), self.safe_candidate_path[k,j])               
        # # Visualize Path in sl coordinate
        # plt.plot(range(0, self.fix_path_point), self.safe_candidate_path[0],'r')
        # plt.plot(range(0, self.fix_path_point), self.safe_candidate_path[1],'r')
        # plt.plot(range(0, self.fix_path_point), self.safe_candidate_path[2],'r')
        # # plt.plot(range(0, self.fix_path_point), self.safe_candidate_path[3],'r')
        # # plt.plot(range(0, self.fix_path_point), self.safe_candidate_path[4],'r')
        # plt.plot(range(0, self.fix_path_point), self.obs_candidate_path[0],'g')
        # plt.plot(range(0, self.fix_path_point), self.obs_candidate_path[1],'g')
        # plt.plot(range(0, self.fix_path_point), self.obs_candidate_path[2],'g')
        # plt.plot(range(0, self.fix_path_point), self.obs_candidate_path[3],'g')
        # plt.plot(range(0, self.fix_path_point), self.obs_candidate_path[4],'g')
        # plt.plot(range(0, self.fix_path_point), self.obs_candidate_path[5],'g')
        # plt.plot(range(0, self.path_point), self.candidate_path[0],'r')
        # plt.plot(range(0, self.path_point), self.candidate_path[1],'r')
        # plt.plot(range(0, self.path_point), self.candidate_path[2],'r')
        # plt.plot(range(0, self.path_point), self.candidate_path[3],'r')
        # plt.plot(range(0, self.path_point), self.candidate_path[4],'r')

        # Visualize Path in tm coordinate
        # plt.plot(self.safe_candidate_path_xy[0, :, 0], self.safe_candidate_path_xy[0, :, 1])
        # plt.plot(self.safe_candidate_path_xy[1, :, 0], self.safe_candidate_path_xy[1, :, 1])
        # plt.plot(self.safe_candidate_path_xy[2, :, 0], self.safe_candidate_path_xy[2, :, 1])
        # plt.plot(self.safe_candidate_path_xy[3, :, 0], self.safe_candidate_path_xy[3, :, 1])
        # plt.plot(self.safe_candidate_path_xy[4, :, 0], self.safe_candidate_path_xy[4, :, 1])
        # plt.plot(BASE_XY[:, 0], BASE_XY[:, 1])
        # plt.show()
        rviz_msg = Marker (
            type = Marker.POINTS,
            lifetime=rospy.Duration(0),
            scale=Vector3(0.5,0.5,0.1),
            header=Header(frame_id='map'),
            color=ColorRGBA(0,1.0,0,1.0),
            id=1
        )
        for j in range(0,self.fix_path_point):
            for i in range(0, PATH_INDEX):
                po=Point()
                po.x = self.safe_candidate_path_xy[i][j][0] - x_offset
                po.y = self.safe_candidate_path_xy[i][j][1] - y_offset
                po.z = 0
                rviz_msg.points.append(po)
                
        self.cand_path.publish(rviz_msg)

        return self.si

    def localize2sl(self,s, x, y):
        # ss = int( s * 1 / N )
        ss = int( s *5 )
        position = [x, y]
        search = 0
        base_length = BASE_FRAME_XY_DT.shape[0]
        diff = 100 # gradient

        # Gradient Descendent method
        while(abs(diff) > 0.001):
            diff = distance.euclidean(position, BASE_FRAME_XY_DT[ss + 1, :]) - \
                   distance.euclidean(position, BASE_FRAME_XY_DT[ss    , :])    # forward difference
            search += 1
            alpha = 1/(1+search/50) # descend rate
            descendent = -alpha*diff

            # Determine the slope
            if -1<descendent and descendent<0: descendent = -1                                                                                                                                                                                                                                                                                 
            elif 0<descendent and descendent<1: descendent = 1
            elif isnan(descendent): descendent = 0
            ss += int(descendent)
            if ss > base_length - 1: # final index
                ss = base_length - 1
                break
            elif ss < 0: # first index
                ss = 0
                break

            # Search only 20 times
            if search == 50:################ path planning 속도 요인 1순위
                break
        # s = int(round(ss/(1/N), ndigits=1) )
        s = int(round(ss/5, ndigits=1) ) 

        # Determine the sign
        base_vec = np.array([cos(BASE_THETA[s]), sin(BASE_THETA[s])])
        pose_vec = np.array([x - BASE_XY[s, 0], y - BASE_XY[s, 1]])
        sign = np.cross(base_vec, pose_vec)
        if sign >= 0: sign = 1 
        else: sign = -1
        q = sign*distance.euclidean(BASE_XY[s], position) 
        return s, q

    def localize2xy(self, s, q):
        x = BASE_XY[s, 0] + q*cos(BASE_THETA[s] + pi/2)
        y = BASE_XY[s, 1] + q*sin(BASE_THETA[s] + pi/2)

        return x, y 

    def safety_cost(self, si): 
        safety_cost = np.zeros(PATH_INDEX)
        safety_cost_filtered = np.zeros(PATH_INDEX)
        obs_s_in = np.array([])
        obs_q_in = np.array([])
        obs_s_, obs_q_ = 0, 0
        obsxy_temp= self.obs_xy



        # Visualize Obstacle in tm coordinate
        rviz_msg = Marker (
                type = Marker.POINTS,
                scale = Vector3 (0.5,0.5,0),
                header = Header(frame_id = 'map'),
                color = ColorRGBA(1.0,0.0,0.0,1.0),
                id = 0
            )
        for i in range (0, self.obs_xy.shape[0]):
            po = Point()
            po.x = self.obs_xy[i][0] - x_offset
            po.y = self.obs_xy[i][1] - y_offset
            po.z = 0
            rviz_msg.points.append(po)
        self.obs_plot.publish(rviz_msg)

        for i in range(0, obsxy_temp.shape[0]):
            obs_s_, obs_q_ = self.localize2sl(si,obsxy_temp[i, 0], obsxy_temp[i, 1])
            obs_s_in = np.append(obs_s_in, obs_s_)
            obs_q_in = np.append(obs_q_in, obs_q_) 

        ###################################################################
        # Check collision 1 (Extend Point Scale)
        # for i in range(0, PATH_INDEX):
        #     for j in range(0, self.fix_path_point): 
        #         for k in range(0, len(obs_s_in)): 
        #             # if obs_q_in[k]
        #             if obs_q_in[k] - 0.5 < self.safe_candidate_path[i, j] and self.safe_candidate_path[i, j] < obs_q_in[k] + 0.5:
        #                 safety_cost[i] = 1
        # # Gaussian Convolution
        # kernel = [0.06136, 0.24477, 0.38774, 0.24477, 0.06136] # �� 1 size 5
        # safety_cost = np.insert(safety_cost,0,safety_cost[0])
        # safety_cost = np.insert(safety_cost,0,safety_cost[0])
        # safety_cost = np.insert(safety_cost,6,safety_cost[6])
        # safety_cost = np.insert(safety_cost,7,safety_cost[7])
        # for i in range(0,PATH_INDEX):
        #     safety_cost_filtered[i] = kernel[0]*safety_cost[i] + kernel[1]*safety_cost[i+1] + kernel[2]*safety_cost[i+2] + kernel[3]*safety_cost[i+3] + kernel[4]*safety_cost[i+4]
        ################################################################### 
        # Check collision 2 (Count Point Number)
        for k in range(0,len(obs_s_in)):
            for j in range(1,self.fix_path_point):
                if obs_s_in[k] == si + j:
                    for i in range(0,PATH_INDEX):
                        if self.obs_candidate_path[i,j]<=obs_q_in[k] and self.obs_candidate_path[i+1,j] > obs_q_in[k]:
                            safety_cost[i]=safety_cost[i]+1
                            
        # # SL visuallization
        # plt.plot(range(si, si+fix_path_point), self.obs_candidate_path[0],'g')
        # plt.plot(range(si, si+fix_path_point), self.obs_candidate_path[1],'g')
        # plt.plot(range(si, si+fix_path_point), self.obs_candidate_path[2],'g')
        # plt.plot(range(si, si+fix_path_point), self.obs_candidate_path[3],'g')
        # plt.plot(range(si, si+fix_path_point), self.obs_candidate_path[4],'g')
        # plt.plot(range(si, si+fix_path_point), self.obs_candidate_path[5],'g')
        # plt.plot(obs_s_in[:],obs_q_in[:],'ro')
        # plt.show()

        # Gaussian Convolution
        kernel = [0.331111, 0.337778, 0.331111] # �� 5 size 3
        safety_cost = np.insert(safety_cost,0,safety_cost[0])
        safety_cost = np.insert(safety_cost,3,safety_cost[3])
        for i in range(0,PATH_INDEX):
            safety_cost_filtered[i] = kernel[0]*safety_cost[i] + kernel[1]*safety_cost[i+1] + kernel[2]*safety_cost[i+2]
        max_cost = np.max(safety_cost_filtered)
        if max_cost != 0:
            for i in range(0,PATH_INDEX):
                safety_cost_filtered[i] = safety_cost_filtered[i]/max_cost
        # plt.plot(range(0,PATH_INDEX),safety_cost_filtered)
        # plt.show()
  ###################################################################
        # Check collsion 3 (Obtimization)
        # obs_data = np.zeros(3)
        # for i in range(0,len(obs_s_in)):
        #     for j in range(0,len(obs_data)):
        #         if obs_q_in[i] < (-1 + 2*j)-INTERVAL and obs_q_in[i] >= (-3 + 2*j)*INTERVAL:
        #             obs_data[j] = obs_data[j] + 1
        
        # if obs_data[1] > 1:
        #     for i in range(0,PATH_INDEX):
        #         safety_cost[i] = 1
        #     safety_cost[1] = 0
    
        # # Gaussian Convolution
        # kernel = [0.27901, 0.44198, 0.27901]
        # safety_cost = np.insert(safety_cost,0,safety_cost[0])
        # safety_cost = np.insert(safety_cost,5,safety_cost[5])
        # for i in range(0,PATH_INDEX):
        #     safety_cost_filtered[i] = kernel[0]*safety_cost[i] + kernel[1]*safety_cost[i+1] + kernel[2]*safety_cost[i+2]

        return safety_cost_filtered

    def offset_cost(self):
        offset_cost = np.zeros(PATH_INDEX)
        for i in range(0, PATH_INDEX * 2 + 1):
            if i%2==1:
                k = int(np.modf([i/2.0])[1][0])
                offset_cost[k] = 1.0*abs(self.qf[i])/(INTERVAL*(PATH_INDEX-1))

        return offset_cost

    def consistency_cost(self, si):
        consistency_cost = np.zeros(PATH_INDEX)
        if self.prev_si + self.path_length - si +1 >=0:
            q_dist = np.zeros(self.prev_si + self.fix_path_point - si) 
            if len(q_dist)<=self.fix_path_point:
                for i in range(0,PATH_INDEX):
                    for j in range(0,len(q_dist)):
                        q_dist[j] = (abs(self.safe_candidate_path[i,j] - self.prev_sl_selected_path[j]))
                        consistency_cost[i] = sum(q_dist)*1.0 / len(q_dist) 
        else : consistency_cost = np.zeros(PATH_INDEX)
        self.prev_si = si
        max_cost = np.max(consistency_cost)
        if max_cost != 0:
            for i in range(0,PATH_INDEX):
                consistency_cost[i] = consistency_cost[i]/max_cost

        return consistency_cost

    def select_path(self):
        si = self.generate_path()
        offset = self.offset_cost()
        safety = self.safety_cost(si)
        consistency = self.consistency_cost(si)
        self.cost = W_OFFSET*offset + W_CONSISTENCY*consistency + W_SAFETY*safety # 
        # Cost Function
        self.selected_index = np.where(self.cost == min(abs(self.cost)))[0][0]
        self.prev_sl_selected_path = self.safe_candidate_path[self.selected_index,:]
        # kappa = self.calculate_kappa(self.selected_index,self.path_point)
        kappa = Float64MultiArray()
        for cur_kappa in BASE_KAPPA[si:si+self.path_point,0]:
            kappa.data.append(cur_kappa)
        goal= self.pub2path_tracker(self.selected_index) # 
        # self.selected_index = self.line_control(selected_index,line_offset,safety_on,goal)       #line_offest 받고 나서 돌려볼 수 있음
        # return self.candidate_path_xy[self.selected_index,:]
        return goal , kappa

    # def line_control(self,selected_index,line_offest,safety_on,goal): # 차선 정보로 selected_index 보정 해주는 함수
    #     #line offest right is minus, left is plus 
    #     if (fabs(line_offest) > INTERVAL) & (safety_on ==0):
    #         if line_offest > 0 :
    #             goal[1] = goal[1] + line_offest
    #     return index

    def calculate_kappa(self,selected_index,path_point): #  
        
        ds = 1.0 # 1meter
        kappa = Float64MultiArray()
        for i in range(1,[path_point]):
            y_1deriv = (self.candidate_path_xy[selected_index,i+1,1] - self.candidate_path_xy[selected_index,i,1]) / ds
            x_1deriv = (self.candidate_path_xy[selected_index,i+1,0] - self.candidate_path_xy[selected_index,i,0]) / ds
            y_2deriv = (self.candidate_path_xy[selected_index,i+1,1] - 2*self.candidate_path_xy[selected_index,i,1] + self.candidate_path_xy[selected_index,i-1,1]) / (ds**2)
            x_2deriv = (self.candidate_path_xy[selected_index,i+1,0] - 2*self.candidate_path_xy[selected_index,i,0] + self.candidate_path_xy[selected_index,i-1,0]) / (ds**2)
            
            kappa_ = abs(x_1deriv*y_2deriv - x_2deriv*y_1deriv) / ((x_1deriv**2 + y_1deriv**2)**1.5)
            kappa.data.append(kappa_)

        y_1deriv0 = (self.candidate_path_xy[selected_index,1,1] - self.candidate_path_xy[selected_index,0,1]) / ds 
        x_1deriv0 = (self.candidate_path_xy[selected_index,1,0] - self.candidate_path_xy[selected_index,0,0]) / ds
        y_2deriv0 = (self.candidate_path_xy[selected_index,2,1] - 2*self.candidate_path_xy[selected_index,1,1] + self.candidate_path_xy[selected_index,0,1]) / (ds**2) 
        x_2deriv0 = (self.candidate_path_xy[selected_index,2,0] - 2*self.candidate_path_xy[selected_index,1,0] + self.candidate_path_xy[selected_index,0,0]) / (ds**2)
        kappa_0 = abs(x_1deriv0*y_2deriv0 - x_2deriv0*y_1deriv0) / ((x_1deriv0**2 + y_1deriv0**2)**1.5)
        kappa.data.insert(0, kappa_0)
        y_1deriv_final = (self.candidate_path_xy[selected_index,-1,1] - self.candidate_path_xy[selected_index,-2,1]) / ds # backward difference
        x_1deriv_final = (self.candidate_path_xy[selected_index,-1,0] - self.candidate_path_xy[selected_index,-2,0]) / ds
        y_2deriv_final = (self.candidate_path_xy[selected_index,-3,1] - 2*self.candidate_path_xy[selected_index,-2,1] + self.candidate_path_xy[selected_index,-1,1]) / (ds**2) # Use sequence -2 2nd derivative
        x_2deriv_final = (self.candidate_path_xy[selected_index,-3,0] - 2*self.candidate_path_xy[selected_index,-2,0] + self.candidate_path_xy[selected_index,-1,0]) / (ds**2)
        kappa_final = abs(x_1deriv_final*y_2deriv_final - x_2deriv_final*y_1deriv_final) / ((x_1deriv_final**2 + y_1deriv_final**2)**1.5)
        kappa.data.append(kappa_final)
        
        return kappa

    def pub2path_tracker(self,selected_index):
        goal = Pose()
        
        # ld = self.ld
        # goal_index = int(ld)
        # goal_n = [self.candidate_path_xy[selected_index,goal_index,0],self.candidate_path_xy[selected_index,goal_index,1] ] # 
        # dist = distance.euclidean(self.current_pose,goal_n) #
        # while dist <= ld :
        #     if goal_index < path_length:
        #         goal_index += 1
        #         dist = distance.euclidean(self.current_pose,goal_n) 
        #         goal_n = [self.candidate_path_xy[selected_index,goal_index,0],self.candidate_path_xy[selected_index,goal_index,1] ]
        #     if goal_index == path_length:
        #         goal_index += 1
        #         break
        
        T = [[cos(self.heading), sin(self.heading), -self.current_pose[0]*cos(self.heading)-self.current_pose[1]*sin(self.heading)],\
            [-sin(self.heading),  cos(self.heading), self.current_pose[0]*sin(self.heading)-self.current_pose[1]*cos(self.heading)],\
            [           0,             0, 1]]
        tm_goal = [self.candidate_path_xy[selected_index,-1,0],self.candidate_path_xy[selected_index,-1,1],1] 
        self._sl=np.dot(T, np.transpose(tm_goal))
        goal.position.x = self._sl[0]
        goal.position.y = self._sl[1]
        #self.goal_pub.publish(goal)
        #self.kappa_pub.publish(kappa)
        return goal

    def visuallizer(self):
        rviz_msg = Marker (
            type = Marker.POINTS,
            lifetime=rospy.Duration(0),
            scale=Vector3(0.5,0.5,0.1),
            header=Header(frame_id='map'),
            color=ColorRGBA(0,1.0,0,1.0),
            id=1
        )
        for i in range(0, self.path_point): 
            for j in range(0, PATH_INDEX):
                po=Point()
                po.x = self.candidate_path_xy[i,j,0]
                po.y = self.candidate_path_xy[i,j,1]
                po.z = 0
                rviz_msg.points.append(po)        
        self.plot_path.publish(rviz_msg)

def main():
    plot_path = rospy.Publisher('/path', Marker, queue_size=1)
    goal_pub = rospy.Publisher('next_goal', Pose, queue_size=1)
    kappa_pub = rospy.Publisher('kappa',Float64MultiArray,queue_size=1)
    rospy.init_node('path_planner')
    
    pth = PathPlanner()
    dataHub = sensor_data_communication.sensor_data_communicationer()
    # self.pub2path_tracker = rospy.Publisher('vel_ld',Vel_ld,queue_size=1)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown() :
        while not dataHub.readySensor() :
            i = 0
        x, y, heading = dataHub.get_current_tm_and_heading()
        ld = ld_planner.setup_ld(dataHub.get_current_car_speed())
        obs_xy = dataHub.get_current_obstacles()
    
        pth.set_current_pose_head(x,y, heading)
        pth.set_current_ld_path_infor(ld)
        pth.set_obs_xy(obs_xy)

        goal, kappa=pth.select_path()
        goal_pub.publish(goal)

        kappa_pub.publish(kappa)
        dataHub.setSensorFlagOff()
        rate.sleep()

if __name__ == '__main__':
    main()