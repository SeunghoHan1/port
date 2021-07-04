#!/usr/bin/env python
#-*-coding:utf-8-*-

def setup_ld(vel): 
    #현재 속력(vel)에 따라서 ld 설정     
    #현재 3m ~ 8m를 적정 ld 범위로 보고 있음      
    v_start = 40 # 50,60,80,50     
    if vel < v_start:         
        ld = 3     
    else:         
        ld = float(vel)/4-7 #속력 범위 40~60         
        # ld = float(vel)/8-3 #속력 범위 50~90         
        # ld = float(vel)/12-2 #속력 범위 60~120         
        # ld = float(vel)/14-2 #속력 범위 80~150         
        # ld = float(vel)/14   #속력 범위 50~120       
    return ld