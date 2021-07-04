#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA, Header, Float64
from tf.transformations import quaternion_from_euler

waypoint = np.load(file='/home/han/catkin_ws/src/macaron_2/path/K-CITY-garage-0.5m.npy')

x_offset = 935650.327283
y_offset = 1916317.270770
class Visualization():
    def __init__(self):
        self.plot = rospy.Publisher('/track', Marker, queue_size=1)
        self.plotpose = rospy.Publisher('/pose',PoseStamped, queue_size=1)

    def global_path(self):
        path1 = np.transpose(waypoint)
        bm=np.zeros((path1.shape[1],2))
        for i in range(0,path1.shape[1]-1):
            bm[i][0]=path1[0][i]-x_offset
            bm[i][1]=path1[1][i]-y_offset
        rviz_msg = Marker(
                type=Marker.LINE_STRIP,
                scale=Vector3(0.1,0.1,0),
                header=Header(frame_id='map'),
                color=ColorRGBA(1.0, 0.0, 0.0, 1.0),
                id=0
            )
            ## plotting code about text, if you want to see realtime see ins_y node
        for i in range(path1.shape[1]):
            po=Point()    
            po.x=bm[i][0]
            po.y=bm[i][1]
            po.z=0
            print(po)
            rviz_msg.points.append(po)
            rospy.sleep(0.01)
            self.plot.publish(rviz_msg)
        

    def presentPOSE(self):
        pose = PoseStamped()
        current_time=rospy.get_rostime()
        while not rospy.is_shutdown():
            heading_sub = rospy.wait_for_message('heading',Float64)
            heading=float(heading_sub.data)
            current_pose = rospy.wait_for_message('current_tm',Pose)
            x=current_pose.position.x
            y=current_pose.position.y
            #print(current_pose)
            pose.pose.position.x = x-x_offset
            pose.pose.position.y = y-y_offset
            ##print('x',pose.pose.position.x,'  y',pose.pose.position.y)
            pose.pose.position.z = 0
            (a,b,c,d)=quaternion_from_euler(0,0,heading)
            pose.pose.orientation.x=a
            pose.pose.orientation.y=b
            pose.pose.orientation.z=c
            pose.pose.orientation.w=d
            #print('a',a,'b',b,'c',c,'d',d)
            pose.header.stamp = current_time
            pose.header.frame_id = "map"
            self.plotpose.publish(pose)

    def run(self):
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            
            rate.sleep()         

def main():
    rospy.init_node('Visuallization')
    Visualization()


if __name__ == '__main__':
    main()