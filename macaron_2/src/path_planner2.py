# #!/usr/bin/env python
# # -*- coding: utf-8 -*-
# """
# Created on Mon Aug  3 18:27:01 2020

# @author: Elitebook 8570w
# """


# import numpy as np
# import copy
# import math
# from scipy.spatial import distance
# import mathdir.cubic_spline_planner as cubic_spline_planner
# import mathdir.polynomial as polynomial

# # Parameter
# MAX_SPEED = 25.0 / 3.6  # maximum speed [m/s]
# MAX_ACCEL = 2.0  # maximum acceleration [m/ss]
# MAX_CURVATURE = 10  # maximum curvature [1/m] 
# MAX_ROAD_WIDTH = 7.0  # maximum road width [m]
# D_ROAD_W = 1.0  # road width sampling length [m]
# DT = 0.5  # time tick [s]
# MAXT = 4.0  # max prediction time [m]
# MINT = 3.0  # min prediction time [m]
# TARGET_SPEED = 15.0 / 3.6  # target speed [m/s]
# D_T_S = 3.0 / 3.6  # target speed sampling length [m/s] 한번에 낼 수 있는 속도
# N_S_SAMPLE = 1  # sampling number of target speed
# ROBOT_RADIUS = 2.0  # robot radius [m] 차의 크기

# # cost weights
# KJ = 0.1
# KT = 0.1
# KD = 1.0
# KLAT = 1.0
# KLON = 1.0

# faTrajX = []
# faTrajY = []
# faTrajCollisionX = []
# faTrajCollisionY = []
# faObCollisionX = []
# faObCollisionY = []
# fpplist = []




# class Frenet_path:

#     def __init__(self):
#         self.t = []
#         self.d = []
#         self.d_d = []
#         self.d_dd = []
#         self.d_ddd = []
#         self.s = []
#         self.s_d = []
#         self.s_dd = []
#         self.s_ddd = []
#         self.cd = 0.0
#         self.cv = 0.0
#         self.cf = 0.0

#         self.x = []
#         self.y = []
#         self.yaw = []
#         self.ds = []
#         self.c = []


# def calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0, target_speed):

#     frenet_paths = []

#     # generate path to each offset goal
#     for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):

#         # Lateral motion planning
#         for Ti in np.arange(MINT, MAXT, DT):
#             fp = Frenet_path()

#             lat_qp = polynomial.quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

#             fp.t = [t for t in np.arange(0.0, Ti, DT)]
#             fp.d = [lat_qp.calc_point(t) for t in fp.t]
#             fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
#             fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
#             fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

#             # Loongitudinal motion planning (Velocity keeping)
#             for tv in np.arange(target_speed - D_T_S * N_S_SAMPLE, target_speed + D_T_S * N_S_SAMPLE, D_T_S):
#                 tfp = copy.deepcopy(fp)
#                 lon_qp = polynomial.quartic_polynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

#                 tfp.s = [lon_qp.calc_point(t) for t in fp.t]
#                 tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
#                 tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
#                 tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

#                 Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
#                 Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

#                 # square of diff from target speed
#                 ds = (target_speed - tfp.s_d[-1])**2

#                 tfp.cd = KJ * Jp + KT * Ti + KD * tfp.d[-1]**2
#                 tfp.cv = KJ * Js + KT * Ti + KD * ds
#                 tfp.cf = KLAT * tfp.cd + KLON * tfp.cv

#                 frenet_paths.append(tfp)

#     return frenet_paths


# def calc_global_paths(fplist, csp):
#     global faTrajX, faTrajY
#     for fp in fplist:

#         # calc global positions
#         for i in range(len(fp.s)):
#             ix, iy = csp.calc_position(fp.s[i])
#             if ix is None:
#                 break
#             iyaw = csp.calc_yaw(fp.s[i])
#             di = fp.d[i]
#             fx = ix + di * math.cos(iyaw + math.pi / 2.0)
#             fy = iy + di * math.sin(iyaw + math.pi / 2.0)
#             fp.x.append(fx)
#             fp.y.append(fy)

#         # Just for plotting
#         faTrajX.append(fp.x)
#         faTrajY.append(fp.y)

#         # calc yaw and ds
#         for i in range(len(fp.x) - 1):
#             dx = fp.x[i + 1] - fp.x[i]
#             dy = fp.y[i + 1] - fp.y[i]
#             fp.yaw.append(math.atan2(dy, dx))
#             fp.ds.append(math.sqrt(dx**2 + dy**2))

#         fp.yaw.append(fp.yaw[-1])
#         fp.ds.append(fp.ds[-1])

#         # calc curvature
#         for i in range(len(fp.yaw) - 1):
#             fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

#     return fplist


# def check_collision(fp, ob):
#     global faTrajCollisionX, faTrajCollisionY, faObCollisionX, faObCollisionY
#     if len(ob) == 0 :
#         return False
#     for i in range(len(ob[:, 0])):
#         d = [((ix - ob[i, 0])**2 + (iy - ob[i, 1])**2)
#              for (ix, iy) in zip(fp.x, fp.y)]

#         collision = any([di <= ROBOT_RADIUS**2 for di in d])

#         if collision:
#             faTrajCollisionX.append(fp.x)       
#             faTrajCollisionY.append(fp.y)

#             if ob[i,0] not in faObCollisionX or ob[i,1] not in faObCollisionY:
#                 faObCollisionX.append(ob[i, 0])
#                 faObCollisionY.append(ob[i, 1])     
#             return True

#     return False


# def check_paths(fplist, ob):

#     okind = []
#     for i in range(len(fplist)):
#         #if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
#         #    continue
#         #elif any([abs(a) > MAX_ACCEL for a in fplist[i].s_dd]):  # Max accel check
#         #    continue
#         #elif any([abs(c) > MAX_CURVATURE for c in fplist[i].c]):  # Max curvature check
#         #    continue
#         if check_collision(fplist[i], ob):
#             continue

#         okind.append(i)

#     return [fplist[i] for i in okind]


# def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob, target_speed):
#     global fpplist
#     fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0, target_speed)
#     fplist = calc_global_paths(fplist, csp)
#     #print('before check',fplist)
#     fplist = check_paths(fplist, ob)


#     fpplist.extend(fplist)
#     # find minimum cost path
#     mincost = float("inf")
#     bestpath = None
#     #print(fplist)
#     for fp in fplist:
#         if mincost >= fp.cf:
#             mincost = fp.cf
#             bestpath = fp
#     return bestpath


# def generate_target_course(x, y):
#     csp = cubic_spline_planner.Spline2D(x, y)
#     s = np.arange(0, csp.s[-1], 1)

#     rx, ry, ryaw, rk = [], [], [], []
#     for i_s in s:
#         ix, iy = csp.calc_position(i_s)
#         rx.append(ix)
#         ry.append(iy)
#         ryaw.append(csp.calc_yaw(i_s))
#         rk.append(csp.calc_curvature(i_s))

#     return rx, ry, ryaw, rk, csp

# class path_planner:

#     def __init__(self, filename="/home/park/catkin_ws/src/macaron_2/path/manhae1.npy"):     
#         base_frame = np.load(file=filename)
#         self.wx = base_frame[0:base_frame.shape[0] -2, 0]
#         self.wy = base_frame[0:base_frame.shape[0] -2,-1]
#         self.obs = []
#         """
#         self.obs = np.array([[955793.371085,1951231.241498],
#                        [955802.831106,1951256.440183],
#                        [955781.147602,1951255.923337],
#                        [955782.256198,1951253.487930],
#                        ])
#         """
#         # 경로 예측에 필요한 초기 변수들
#         self.c_speed = 0 / 3.6  # current speed [m/s] 
#         self.c_d = 0.0  # current lateral position [m] offset값
#         self.c_d_d = 0.0  # current lateral speed [m/s]
#         self.c_d_dd = 0.0  # current latral acceleration [m/s]
#         self.s0 = 0.0  # current course position 초기 위치(currnet_tm)
#         tx, ty, tyaw, tc, csp = generate_target_course(self.wx, self.wy)
#         self.tx = tx
#         self.ty = ty
#         self.tyaw = tyaw
#         self.tc = tc
#         self.csp = csp
#         self.target_speed = TARGET_SPEED
#         #print("generate_target_course",self.tx)
        
        
    
#     #변수 설정 함수들
#     # obstacle의 원소는 [x,y]의 튜플형태를 따른다.
#     def putObstacle(self,ob):
#         np.append(self.obs, ob)
#     def setObstacles(self,obs):
#         self.obs = obs
#     def getObstacles(self):
#         return self.obs
#     def getGoalXY(self):
#         return self.tx[-1], self.ty[-1]
#     def getTxTy(self):
#         return self.tx, self.ty
#     def lowerTargetSpeed(self,velocity):
#         self.target_speed -= velocity / 3.6
#     def higherTargetSpeed(self,velocity):
#         self.target_speed += velocity / 3.6
#     def setTargetSpeed(self, target_speed):
#         self.target_speed = target_speed / 3.6
#     def setCurrentS_Position(self, s0):
#         self.s0 = s0
#     #def removeObstacle(ob):
#     #    np.delete
#     def getClosestSPoint(self, cart_vx, cart_vy):
#         n_closestrefpoint = 0
#         mindistance = 999

#         for i in range(len(self.csp.s)):
#             t_distance = math.sqrt((cart_vx - self.csp.sx.y[i]) ** 2 + (cart_vy - self.csp.sy.y[i]) ** 2)
#             if t_distance < mindistance :
#                 mindistance = t_distance
#                 n_closestrefpoint = i
#             else :
#                 continue
#         return self.csp.s[n_closestrefpoint]

#     def select_path(self,target_speed):
#         #print("select path",self.csp, self.s0, self.c_speed, self.c_d, self.c_d_d, self.c_d_dd, self.obs)
#         path = frenet_optimal_planning(self.csp, self.s0, self.c_speed, self.c_d, self.c_d_d, self.c_d_dd, self.obs, target_speed / 3.6)
#         return path
    
#     def updatePlanningParams(self,s0, c_d, c_d_d, c_d_dd, c_speed):
#         self.s0 = s0
#         self.c_d = c_d
#         self.c_d_d = c_d_d
#         self.c_d_dd = c_d_dd
#         self.c_speed = c_speed
#         #if path_len > 48:
#         #    self.lowerTargetSpeed(D_T_S)
        
    
    
    
# def main():

#     print(__file__ + "start!!")
#     #as
#     # way points
#     #wx = [0.0, 10.0, 20.5, 35.0, 70.5]
#     #wy = [0.0, -6.0, 5.0, 6.5, 0.0]
#     # obstacle lists
#     #ob = np.array([[955793.371085,1951231.241498],
#     #               [955802.831106,1951256.440183],
#     #               [955781.147602,1951255.923337],
#     #               [955782.256198,1951253.487930],
#     #               ])
    


#     # initial state
#     pthplanner = path_planner("/home/park/catkin_ws/src/macaron_2/path/manhae1.npy")
#     #for i in range(500): # search depth
#     i = 0
#     while True :
#         i = i + 1
#         print("rospy is running")
#         path = pthplanner.select_path(15.0/3.6)
#         if path is None:
#             print("no more path generated")
#             break
#         #print(path)
#         s0 = path.s[1]
#         c_d = path.d[1]
#         c_d_d = path.d_d[1]
#         c_d_dd = path.d_dd[1]
#         c_speed = path.s_d[1]
#         print(path.s_d)
#         pthplanner.updatePlanningParams(s0, c_d, c_d_d, c_d_dd, c_speed)
        
#         # goal check
#         goalx, goaly = pthplanner.getGoalXY()
#         #tx, ty = pthplanner.getTxTy()
#         #print("goal info", path.x[1], goalx, path.y[1], goaly)
#         print("c_speed",c_speed*3.6)
#         if np.hypot(path.x[1] - goalx, path.y[1] - goaly) <= 1:
#             print("Goal")
#             break

#         # animation part
        

#     print("Finish")

# if __name__ == '__main__':
#     main()
    
    